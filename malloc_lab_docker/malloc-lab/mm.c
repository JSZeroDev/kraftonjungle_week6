/*
 * mm.c - 묵시적 가용 리스트(Implicit Free List)를 사용하는 동적 메모리 할당기
 *
 * [전체 구조]
 *
 * 각 일반 블록은 다음과 같은 형태로 관리한다.
 *
 *                 bp (Block Pointer)
 *                  ↓
 *        ┌────────┬───────────────┬────────┐
 *        │ Header │    Payload    │ Footer │
 *        └────────┴───────────────┴────────┘
 *            4B                       4B
 *
 * - bp는 Header가 아니라 Payload의 시작 주소를 가리킨다.
 * - Header/Footer에는 "블록 전체 크기 + 할당 여부(alloc bit)"를 저장한다.
 * - 블록 전체 크기에는 Header와 Footer도 포함된다.
 * - 블록 크기는 8바이트 정렬을 유지한다.
 *
 * [현재 구현 방식]
 *
 * 1. 묵시적 가용 리스트(Implicit Free List)
 *    - free 블록만 따로 연결하지 않는다.
 *    - Heap에 존재하는 모든 블록을 물리적인 순서대로 탐색한다.
 *    - Header에 저장된 block size를 이용해 다음 블록으로 이동한다.
 *
 * 2. 배치 정책(Placement Policy)
 *    - First Fit / Next Fit / Best Fit을 각각 구현하였다.
 *    - 현재 find_fit()에서 상황에 따라 선택하여 사용하고 있다.
 *
 * 3. 분할(Splitting)
 *    - 찾은 free block이 필요한 크기보다 충분히 크다면
 *      필요한 부분만 allocated block으로 만들고 나머지는 free block으로 남긴다.
 *
 * 4. 병합(Coalescing)
 *    - 블록이 free될 때 물리적으로 인접한 free block을 즉시 병합한다.
 *
 * 5. Heap 확장
 *    - 적절한 free block을 찾지 못하면 mem_sbrk()를 이용하여 Heap을 확장한다.
 *
 * 6. realloc
 *    - 현재 블록이 충분히 크면 같은 위치에서 유지하거나 남는 공간을 분할한다.
 *    - 더 큰 공간이 필요하고 오른쪽 블록이 free이면 두 블록을 합쳐
 *      제자리에서 확장하며, 남는 공간이 충분하면 다시 free block으로 분할한다.
 *    - 제자리 확장이 불가능하면 새 블록을 할당하고 기존 데이터를 복사한 뒤
 *      기존 블록을 해제한다.
 *    - 왼쪽 free block 활용, 양쪽 동시 확장, Epilogue 직접 확장은 구현하지 않았다.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "mm.h"
#include "memlib.h"

/* 팀 정보 */
team_t team = {
    /* 팀 이름 */
    "Jungle-12",
    /* 첫 번째 팀원 이름 */
    "Joung Seong Young",
    /* 첫 번째 팀원 이메일 */
    "sam12057@gmail.com",
    /* 두 번째 팀원 이름 (없으면 빈 문자열) */
    "Yang Woong Jin",
    /* 두 번째 팀원 이메일 (없으면 빈 문자열) */
    "woong501298@gmail.com"
};

/* ============================================================
 * 기본 상수
 * ============================================================
 *
 * WSIZE = 4B
 *   Header/Footer 한 개의 크기
 *
 * DSIZE = 8B
 *   Double Word 크기이며 현재 allocator의 정렬 기준으로 사용한다.
 *
 * CHUNKSIZE = 256B
 *   초기 Heap을 확장할 때 사용하는 크기
 *
 * 현재 extend_heap()은 바이트 단위 크기를 받는다.
 * 256B부터 4096B까지 비교한 결과, 256B에서 전체 utilization이
 * 가장 높았으므로 초기 확장 크기로 선택하였다.
 */
#define WSIZE       4
#define DSIZE       8
#define CHUNKSIZE (1 << 8)

#define MAX(x, y) ((x) > (y) ? (x) : (y))

/* ============================================================
 * Header / Footer에 size와 alloc bit 저장
 * ============================================================
 *
 * 블록 크기는 8바이트 단위로 정렬되므로 정상적인 size의 하위 3비트는 0이다.
 *
 * 예:
 *      size = 24 = 000...11000
 *
 * 따라서 비어 있는 하위 비트 중 bit 0을 할당 여부 저장에 사용할 수 있다.
 *
 *      alloc = 0 → free
 *      alloc = 1 → allocated
 *
 * 예:
 *      PACK(24, 0) = 24
 *      PACK(24, 1) = 25
 *
 * 즉 Header/Footer 하나에 [block size | alloc bit]를 함께 저장한다.
 */
#define PACK(size, alloc) ((size) | (alloc))

/* ============================================================
 * 메모리 읽기 / 쓰기
 * ============================================================
 *
 * p가 가리키는 위치의 4바이트(WSIZE)를 unsigned int로 읽거나 쓴다.
 *
 * GET(p)      → p 주소에 저장된 4바이트 값을 읽음
 * PUT(p, val) → p 주소에 val을 4바이트 값으로 기록
 *
 * Header/Footer가 4B이기 때문에 이 매크로를 사용한다.
 */
#define GET(p)      (*(unsigned int *)(p))
#define PUT(p, val) (*(unsigned int *)(p) = (val))

/* ============================================================
 * Header/Footer에서 size와 alloc bit 분리
 * ============================================================
 *
 * Header에는 size와 alloc 정보가 같이 들어 있으므로 필요한 정보만
 * 비트 연산으로 분리한다.
 *
 * GET_SIZE:
 *   ~0x7 = ...11111000
 *   하위 3비트를 제거하여 block size만 얻는다.
 *
 * GET_ALLOC:
 *   0x1과 AND하여 가장 마지막 bit(alloc bit)만 얻는다.
 */
#define GET_SIZE(p)  (GET(p) & ~0x7)
#define GET_ALLOC(p) (GET(p) & 0x1)

/* ============================================================
 * bp로부터 Header / Footer 주소 계산
 * ============================================================
 *
 *                 bp
 *                  ↓
 *        ┌────────┬───────────────┬────────┐
 *        │ Header │    Payload    │ Footer │
 *        └────────┴───────────────┴────────┘
 *
 * bp는 Payload의 시작 주소이다.
 *
 * HDRP(bp): bp에서 WSIZE(4B)만큼 앞으로 이동하면 Header
 * FTRP(bp): bp에서 block size만큼 이동한 뒤 DSIZE(8B)를 빼면 Footer
 *
 * 주소 계산을 위해 bp를 char *로 변환한다. char는 1바이트이므로
 * char *의 +1은 주소를 정확히 1바이트 이동시킨다.
 */
#define HDRP(bp) ((char *)(bp) - WSIZE)
#define FTRP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)

/* ============================================================
 * 다음 / 이전 물리적 블록 계산
 * ============================================================
 *
 * Implicit Free List에는 별도의 next/prev 포인터가 없다.
 * 따라서 Header/Footer에 기록된 block size를 이용하여 이동한다.
 *
 * NEXT_BLKP:
 *   현재 block의 size만큼 앞으로 이동
 *
 * PREV_BLKP:
 *   현재 bp 바로 앞에는 이전 block의 Footer가 있으므로
 *   그 Footer에서 이전 block size를 얻은 뒤 그만큼 뒤로 이동
 */
#define NEXT_BLKP(bp) \
    ((char *)(bp) + GET_SIZE(((char *)(bp) - WSIZE)))

#define PREV_BLKP(bp) \
    ((char *)(bp) - GET_SIZE(((char *)(bp) - DSIZE)))

/* ============================================================
 * 요청 크기를 실제 블록 크기로 변환
 * ============================================================
 *
 * 사용자가 요청한 payload 크기(size)에 Header/Footer 공간을 고려하고,
 * 전체 block 크기가 8바이트 배수가 되도록 올림한다.
 *
 * 예:
 *   size = 1  → asize = 16
 *   size = 8  → asize = 16
 *   size = 13 → asize = 24
 *   size = 20 → asize = 32
 *
 * mm_malloc(), mm_realloc()에서 동일한 크기 계산을 사용하기 위해
 * 매크로로 정의한다.
 */
#define ADJUST_SIZE(size) (((size) + 2 * DSIZE - 1) & ~0x7)

/* ============================================================
 * 전역 포인터
 * ============================================================
 *
 * heap_listp
 *   Heap 탐색의 기준이 되는 포인터. 초기화 후 Prologue block 쪽을 가리킨다.
 *
 * rover
 *   Next Fit에서 "지난번 탐색 위치"를 기억하기 위한 포인터.
 *
 * char *를 사용하는 이유:
 *   Heap 내부에서는 주소를 바이트 단위로 계산하는 일이 많기 때문이다.
 *   char *p에서 p + 1은 1바이트 이동하지만, int *p에서 p + 1은
 *   일반적으로 sizeof(int)인 4바이트를 이동한다.
 */
static char *heap_listp = NULL;
static char *rover = NULL;

/* ============================================================
 * 내부 Helper 함수 선언
 * ============================================================
 *
 * static:
 *   이 함수들을 mm.c 내부에서만 사용한다는 의미.
 *
 * void:
 *   반환값이 없음.
 *
 * void *:
 *   특정 자료형을 지정하지 않은 범용 포인터를 반환한다.
 *   즉 여기서는 주로 "블록의 주소"를 반환한다.
 *
 * size_t:
 *   메모리 크기나 배열 크기처럼 음수가 될 수 없는 크기 값을
 *   표현할 때 사용하는 자료형.
 *
 * extend_heap / coalesce / find_fit:
 *   작업 결과로 블록의 위치를 알려줘야 하므로 void * 반환
 *
 * place:
 *   전달받은 블록의 Header/Footer를 직접 수정하기만 하면 되므로
 *   별도의 반환값이 필요하지 않아 void 반환
 */
static void *extend_heap(size_t size);
static void *coalesce(void *bp);
static void *find_fit_first(size_t asize);
static void *find_fit_next(size_t asize);
static void *find_fit_best(size_t asize);
static void *find_fit(size_t asize);
static void place(void *bp, size_t asize);

/*
 * mm_init - malloc allocator가 사용할 초기 Heap 구조를 만든다.
 *
 * 초기에는 다음 16바이트를 만든다.
 *
 *   ┌─────────┬────────────┬────────────┬────────────┐
 *   │ Padding │ Prologue H │ Prologue F │ Epilogue H │
 *   └─────────┴────────────┴────────────┴────────────┘
 *       4B          4B           4B           4B
 *
 * Padding:
 *   이후 실제 payload의 8바이트 정렬을 맞추기 위한 공간.
 *
 * Prologue:
 *   실제 사용자 블록이 아닌 특수한 allocated block.
 *   Heap의 시작 경계에서 coalesce할 때 별도 예외 처리를 줄여준다.
 *
 * Epilogue:
 *   Heap의 끝을 표시하는 특수 Header.
 *   size = 0, alloc = 1이며 Footer/Payload는 없다.
 */
int mm_init(void)
{
    /*
     * mem_sbrk(4 * WSIZE) = 16바이트 확보
     *
     * mem_sbrk()는 성공하면 새 공간의 시작 주소,
     * 즉 확장하기 전의 program break 위치를 반환한다.
     * 실패하면 (void *)-1을 반환한다.
     */
    if ((heap_listp = mem_sbrk(4 * WSIZE)) == (void *)-1)
        return -1;

    PUT(heap_listp, 0);                             /* 정렬용 Padding */
    PUT(heap_listp + (1 * WSIZE), PACK(DSIZE, 1)); /* Prologue Header */
    PUT(heap_listp + (2 * WSIZE), PACK(DSIZE, 1)); /* Prologue Footer */
    PUT(heap_listp + (3 * WSIZE), PACK(0, 1));     /* Epilogue Header */

    /*
     * heap_listp를 Prologue Footer 위치로 이동한다.
     *
     * 일반적인 bp가 Payload 시작을 가리키는 것과 약간 달리,
     * Prologue는 실제 Payload가 없는 특수 블록이다.
     * 이 위치를 기준으로 NEXT_BLKP()를 사용하면 첫 실제 블록으로 이동할 수 있다.
     */
    heap_listp += (2 * WSIZE);

    /*
     * 빈 Heap에 CHUNKSIZE(256B) 크기의 free block을 추가한다.
     */
    if (extend_heap(CHUNKSIZE) == NULL)
        return -1;

    /*
     * Next Fit 정책으로 전환할 때 사용할 rover를 첫 번째 실제 블록 위치로 초기화한다.
     */
    rover = NEXT_BLKP(heap_listp);
    return 0;
}

/*
 * extend_heap - Heap을 size 바이트만큼 확장하여 새로운 free block을 만든다.
 *
 * mem_sbrk(size)는 성공하면 "확장 전 break" 주소를 반환한다.
 * 기존 Epilogue Header가 있던 위치 바로 다음이 새로운 공간이 되므로,
 * HDRP(bp)는 기존 Epilogue 위치를 새로운 free block의 Header로 사용하게 된다.
 *
 * 확장 후 구조:
 *
 *   ... 기존 블록 ...
 *   ┌────────┬────────────────────┬────────┬────────────┐
 *   │ Header │      Payload       │ Footer │ Epilogue H │
 *   └────────┴────────────────────┴────────┴────────────┘
 *      free                          free      size=0/alloc=1
 *
 * 마지막에 coalesce()를 호출하는 이유:
 * Heap 확장 직전의 마지막 블록이 free 상태라면 새 free block과
 * 물리적으로 인접하므로 하나의 큰 free block으로 합칠 수 있기 때문이다.
 *
 * coalesce()가 최종 병합된 block의 bp를 반환하므로
 * extend_heap()도 그 주소를 그대로 호출자에게 반환한다.
 */
static void *extend_heap(size_t size)
{
    char *bp;

    if ((bp = mem_sbrk(size)) == (void *)-1)
        return NULL;

    /* 새 free block의 Header */
    PUT(HDRP(bp), PACK(size, 0));

    /* 새 free block의 Footer */
    PUT(FTRP(bp), PACK(size, 0));

    /* Heap의 새로운 끝을 나타내는 Epilogue Header */
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1));

    return coalesce(bp);
}

/*
 * coalesce - 현재 free block과 물리적으로 인접한 free block을 병합한다.
 *
 * 병합 가능 여부는 이전 블록과 다음 블록의 alloc bit를 확인하여 결정한다.
 *
 * 총 4가지 경우:
 *
 * Case 1
 *   [ALLOC][FREE(bp)][ALLOC]
 *   → 병합하지 않음
 *
 * Case 2
 *   [ALLOC][FREE(bp)][FREE]
 *   → 현재 + 다음 병합
 *
 * Case 3
 *   [FREE][FREE(bp)][ALLOC]
 *   → 이전 + 현재 병합
 *   → 병합된 block의 시작 위치가 이전 block으로 이동하므로 bp도 변경
 *
 * Case 4
 *   [FREE][FREE(bp)][FREE]
 *   → 이전 + 현재 + 다음 모두 병합
 *
 * Footer가 필요한 중요한 이유 중 하나가 바로 이전 블록의 정보를
 * 확인하여 backward coalescing을 할 수 있게 하기 위해서이다.
 *
 * 반환값:
 *   병합이 끝난 최종 free block의 Payload 시작 주소(bp)
 */
static void *coalesce(void *bp)
{
    /*
     * 이전 block의 Footer를 통해 이전 block의 alloc 상태 확인
     * 다음 block은 Header를 통해 alloc 상태 확인
     */
    size_t prev_alloc = GET_ALLOC(FTRP(PREV_BLKP(bp)));
    size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp)));

    /* 현재 block 전체 크기 */
    size_t size = GET_SIZE(HDRP(bp));

    /* Case 1: 이전 allocated, 다음 allocated */
    if (prev_alloc && next_alloc){
        return bp;
    }

    /* Case 2: 이전 allocated, 다음 free */
    else if (prev_alloc && !next_alloc) {
        /* 현재 block 크기 + 다음 free block 크기 */
        size += GET_SIZE(HDRP(NEXT_BLKP(bp)));

        /* 합쳐진 block의 새 Header/Footer 기록 */
        PUT(HDRP(bp), PACK(size, 0));
        PUT(FTRP(bp), PACK(size, 0));
    }

    /* Case 3: 이전 free, 다음 allocated */
    else if (!prev_alloc && next_alloc) {
        /* 이전 free block 크기 + 현재 block 크기 */
        size += GET_SIZE(HDRP(PREV_BLKP(bp)));

        /*
         * 병합 후 Footer는 현재 block의 기존 Footer 위치에 그대로 존재한다.
         * Header는 이전 block의 Header 위치에 기록한다.
         */
        PUT(FTRP(bp), PACK(size, 0));
        PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));

        /*
         * 병합된 block은 이전 block 위치에서 시작하므로
         * bp 역시 이전 block의 Payload 시작 주소로 변경한다.
         */
        bp = PREV_BLKP(bp);
    }

    /* Case 4: 이전 free, 다음 free */
    else {
        /* 이전 + 현재 + 다음 block 크기 */
        size += GET_SIZE(HDRP(NEXT_BLKP(bp)))
              + GET_SIZE(HDRP(PREV_BLKP(bp)));

        /*
         * 가장 앞쪽인 이전 block의 Header와
         * 가장 뒤쪽인 다음 block의 Footer를 새 경계로 사용한다.
         */
        PUT(HDRP(PREV_BLKP(bp)), PACK(size, 0));
        PUT(FTRP(NEXT_BLKP(bp)), PACK(size, 0));

        /* 병합 후 시작 위치는 이전 block */
        bp = PREV_BLKP(bp);
    }
        /*
     * 병합으로 기존 블록 경계가 사라지면서 rover가 최종 free block의
     * 내부를 가리키게 된 경우, 병합된 블록의 시작점으로 보정한다.
     *
     * rover == bp이면 이미 유효한 현재 블록의 시작점이고,
     * rover == NEXT_BLKP(bp)이면 유효한 다음 블록의 시작점이므로
     * 두 경계는 제외하고 그 사이에 있을 때만 보정한다.
     */
    if ( rover && (char *)bp < rover && rover < NEXT_BLKP(bp)){
        rover = bp;
    }
    return bp;
}

/*
 * find_fit_first - First Fit 방식으로 들어갈 수 있는 첫 번째 free block을 찾는다.
 *
 * First Fit:
 *   Heap 앞쪽부터 순서대로 탐색하다가
 *   asize 이상인 첫 번째 free block을 즉시 선택한다.
 *
 * Implicit Free List이므로 free block만 탐색하는 것이 아니라
 * allocated/free 여부와 관계없이 모든 물리적 block을 지나간다.
 *
 * Header의 size가 0인 블록은 Epilogue이므로 탐색을 종료한다.
 *
 * 반환값:
 *   적절한 free block을 찾으면 해당 block의 bp
 *   찾지 못하면 NULL
 */
static void *find_fit_first(size_t asize)
{
    char *bp = heap_listp;

    /*
     * NEXT_BLKP(bp)를 통해 block size만큼씩 이동한다.
     * GET_SIZE(...) == 0이면 Epilogue에 도착한 것이다.
     */
    for (; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp)) {
        /* allocated block이면 사용할 수 없으므로 건너뜀 */
        if (GET_ALLOC(HDRP(bp))) {
            continue;
        }

        size_t size = GET_SIZE(HDRP(bp));

        /* 요청 block 크기(asize)를 수용할 수 있는 첫 free block */
        if (size >= asize)
            return bp;
    }

    return NULL;
}

/*
 * find_fit_next - Next Fit 방식으로 들어갈 수 있는 free block을 찾는다.
 *
 * Next Fit:
 *   매번 Heap 처음부터 탐색하지 않고,
 *   이전 탐색 위치(rover)부터 탐색을 시작한다.
 *
 * 탐색 순서:
 *
 *   rover → Heap 끝(Epilogue)
 *               ↓
 *   Heap 처음 → 기존 rover 위치
 *
 * 즉 끝까지 적절한 block을 찾지 못하면 Heap 앞쪽으로 돌아와
 * 원래 rover 위치까지 다시 탐색한다.
 *
 * 반환값:
 *   적절한 free block의 bp
 *   찾지 못하면 NULL
 */
static void *find_fit_next(size_t asize)
{
    char *bp = rover;

    /*
     * 탐색을 시작했을 당시의 rover를 저장한다.
     * Heap 끝까지 갔다가 처음으로 돌아왔을 때
     * 어디까지 탐색해야 하는지 판단하기 위해 사용한다.
     */
    char *old_rover = rover;

    /* 1차 탐색: 현재 rover → Epilogue */
    for (; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp)) {
        if (GET_ALLOC(HDRP(bp))) {
            continue;
        }

        size_t size = GET_SIZE(HDRP(bp));

        if (size >= asize) {
            /*
             * 다음 탐색은 현재 찾은 block 다음부터 시작하도록
             * rover 위치를 갱신한다.
             */
            rover = NEXT_BLKP(bp);
            return bp;
        }
    }

    /*
     * Heap 끝까지 찾지 못했다면
     * Prologue 다음의 첫 실제 block부터 다시 탐색한다.
     */
    bp = NEXT_BLKP(heap_listp);

    /* 2차 탐색: Heap 처음 → 기존 rover */
    for (; bp != old_rover; bp = NEXT_BLKP(bp)) {
        if (GET_ALLOC(HDRP(bp))) {
            continue;
        }

        size_t size = GET_SIZE(HDRP(bp));

        if (size >= asize) {
            rover = NEXT_BLKP(bp);
            return bp;
        }
    }

    return NULL;
}

/*
 * find_fit_best - Best Fit 방식으로 가장 작은 적합 free block을 찾는다.
 *
 * Best Fit:
 *   asize 이상인 free block 중에서 가장 작은 block을 선택한다.
 *
 * First Fit과 달리 처음 발견했다고 바로 반환하면 안 된다.
 * Heap 끝까지 탐색하면서 현재까지 발견한 최적 후보(best)를 기억해야 한다.
 *
 * 장점:
 *   요청 크기에 가까운 block을 선택하여 남는 공간을 줄일 가능성이 있다.
 *
 * 단점:
 *   일반적으로 Heap 전체를 더 많이 탐색하므로 탐색 시간이 증가할 수 있다.
 *
 * 반환값:
 *   최적의 free block을 찾으면 해당 bp
 *   없으면 NULL
 */
static void *find_fit_best(size_t asize)
{
    char *bp = heap_listp;

    /* 현재까지 찾은 가장 좋은 free block */
    char *best = NULL;

    /*
     * 아직 후보가 없으므로 size_t의 최댓값으로 시작한다.
     * 첫 번째 적합 free block은 항상 이 값보다 작으므로 best가 갱신된다.
     */
    size_t best_size = SIZE_MAX;

    for (; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp)) {
        if (GET_ALLOC(HDRP(bp))) {
            continue;
        }

        size_t size = GET_SIZE(HDRP(bp));

        /* 일단 요청 크기를 수용할 수 있어야 후보가 된다. */
        if (size >= asize) {
            /*
             * 아직 후보가 없거나,
             * 현재 block이 기존 best보다 작다면 best 갱신
             */
            if (size < best_size) {
                best_size = size;
                best = bp;
            }
        }
    }

    return best;
}

/*
 * find_fit - 실제 mm_malloc()에서 사용할 배치 정책을 선택한다.
 *
 * 정책을 비교하고 싶다면 아래 반환 함수를 하나씩 변경하여
 * 같은 조건에서 mdriver의 utilization / throughput을 비교할 수 있다.
 *
 *      return find_fit_first(asize);
 *      return find_fit_next(asize);
 *      return find_fit_best(asize);
 */
static void *find_fit(size_t asize)
{
    return find_fit_best(asize);
}

/*
 * place - 선택한 free block에 요청 block을 배치한다.
 *
 * current_size:
 *   선택된 free block의 전체 크기
 *
 * asize:
 *   실제로 할당해야 하는 block 전체 크기
 *   (사용자가 요청한 payload 크기 자체가 아님)
 *
 * re_adjust_size:
 *   할당하고 남는 크기
 *
 *        current_size
 *   ┌─────────────────────────────┐
 *   │          FREE BLOCK         │
 *   └─────────────────────────────┘
 *
 *                ↓ split
 *
 *   ┌───────────────┬─────────────┐
 *   │   ALLOCATED   │    FREE     │
 *   │     asize     │ remainder   │
 *   └───────────────┴─────────────┘
 *
 * 남은 공간이 최소 block 크기인 16B 이상일 때만 split한다.
 *
 * 왜 최소 16B인가?
 *   현재 구조에서는 block이 Header/Footer를 포함하면서
 *   8바이트 정렬을 유지해야 하므로 너무 작은 조각을 별도 free block으로
 *   만들지 않는다.
 */
static void place(void *bp, size_t asize)
{
    /* 현재 선택한 free block의 전체 크기 */
    size_t current_size = GET_SIZE(HDRP(bp));

    /* 할당 후 남게 되는 크기 */
    size_t re_adjust_size = current_size - asize;

    /*
     * 남은 공간이 최소 block 크기(16B) 이상이면 split
     */
    if (re_adjust_size >= 16) {
        /*
         * 중요:
         * 이 시점의 FTRP(bp)는 아직 Header에 current_size가 들어 있으므로
         * 원래 free block의 맨 마지막 Footer를 가리킨다.
         *
         * 이 Footer는 split 후 "남은 free block의 Footer"가 된다.
         */
        PUT(FTRP(bp), PACK(re_adjust_size, 0));

        /* 앞부분을 allocated block으로 변경 */
        PUT(HDRP(bp), PACK(asize, 1));

        /*
         * allocated block의 새 Footer.
         *
         * Header 시작 주소 + asize - WSIZE
         * 위치가 allocated block의 Footer가 된다.
         */
        PUT(FTRP(bp), PACK(asize, 1));

        /*
         * allocated block 바로 다음 위치에
         * 남은 free block의 새 Header를 만든다.
         */
        PUT(HDRP(bp) + asize, PACK(re_adjust_size, 0));
    }
    else {
        /*
         * 남은 공간이 16B보다 작다면 split하지 않고
         * current block 전체를 allocated 상태로 사용한다.
         *
         * 여기서 asize가 아니라 current_size를 기록해야 한다.
         *
         * 예:
         *      current_size = 32
         *      asize        = 24
         *      remainder    = 8
         *
         * 8B는 독립적인 free block으로 사용할 수 없으므로
         * 32B 전체를 하나의 allocated block으로 사용해야 한다.
         *
         * 만약 Header/Footer에 24를 기록하면 실제 block 경계와
         * metadata가 서로 달라져 다음 block 계산이 잘못될 수 있다.
         */
        PUT(HDRP(bp), PACK(current_size, 1));
        PUT(FTRP(bp), PACK(current_size, 1));
    }
}

/*
 * mm_malloc - 사용자가 요청한 size 바이트를 저장할 block을 할당한다.
 *
 * 전체 흐름:
 *
 *      사용자 요청 size
 *             ↓
 *      asize 계산
 *             ↓
 *      find_fit(asize)
 *          /       \
 *       찾음       못 찾음
 *        ↓             ↓
 *      place       extend_heap
 *                       ↓
 *                    place
 *             ↓
 *        Payload 시작 주소 반환
 *
 * size:
 *   사용자가 요청한 Payload 크기
 *
 * asize:
 *   allocator가 실제로 확보해야 하는 전체 block 크기
 *   Header + Payload + Footer + Alignment를 고려한 값
 */
void *mm_malloc(size_t size)
{
    size_t asize;
    void *bp;

    /*
     * 0바이트 요청에는 할당할 block이 없으므로 NULL 반환
     */
    if (size == 0) {
        return NULL;
    }

    /*
     * 요청 size를 실제 block 크기(asize)로 변환한다.
     *
     * 현재 식:
     *
     *      (size + 15) & ~0x7
     *
     * 의미:
     *   Header/Footer 등의 overhead를 포함하면서
     *   최종 block size를 8바이트 배수로 올림한다.
     *
     * 예:
     *      malloc(1)  → asize 16
     *      malloc(8)  → asize 16
     *      malloc(13) → asize 24
     *      malloc(20) → asize 32
     */
    asize = ADJUST_SIZE(size);

    /*
     * 현재 선택된 Fit 정책을 사용하여
     * asize를 수용할 free block을 찾는다.
     */
    bp = find_fit(asize);

    /* 적절한 free block을 찾은 경우 */
    if (bp != NULL) {
        place(bp, asize);
        return bp;
    }

    /*
     * 적절한 free block이 없다면 Heap을 확장한다.
     *
     * 현재 구현은 필요한 asize만큼 Heap을 확장한다.
     * 이후 성능 최적화 단계에서 Heap 확장 정책은 별도로 비교할 수 있다.
     */
    else {
        bp = extend_heap(asize);

        if (bp != NULL) {
            place(bp, asize);
            return bp;
        }

        /* Heap 확장에도 실패했다면 NULL */
        return bp;
    }
}

/*
 * mm_free - allocated block을 free 상태로 변경하고 인접 free block과 병합한다.
 *
 * bp는 mm_malloc()이 반환했던 Payload 시작 주소이다.
 *
 *        bp
 *         ↓
 *   ┌────────┬───────────────┬────────┐
 *   │ Header │    Payload    │ Footer │
 *   └────────┴───────────────┴────────┘
 *
 * free 과정:
 *
 *   1. Header에서 현재 block의 전체 크기를 얻는다.
 *   2. Header/Footer의 alloc bit를 0으로 변경한다.
 *   3. coalesce()를 호출하여 인접한 free block이 있다면 병합한다.
 *
 * mm_free의 반환형이 void인 이유:
 *   free 작업을 수행하면 끝이며 호출자에게 block 주소 등의
 *   결과값을 돌려줄 필요가 없기 때문이다.
 *
 * coalesce() 자체는 병합 후 bp를 반환하지만,
 * mm_free에서는 그 반환값이 필요하지 않으므로
 *
 *      coalesce(bp);
 *
 * 처럼 함수만 실행하고 반환값은 사용하지 않는다.
 */
void mm_free(void *bp)
{
    /* Header에서 alloc bit를 제외한 block 전체 크기를 얻는다. */
    size_t size = GET_SIZE(HDRP(bp));

    /* 같은 block size를 유지하면서 alloc bit만 0으로 변경 */
    PUT(HDRP(bp), PACK(size, 0));
    PUT(FTRP(bp), PACK(size, 0));

    /* 인접 free block이 있다면 병합 */
    coalesce(bp);
}

/*
 * mm_realloc - 기존 데이터를 유지하면서 block의 크기를 변경한다.
 *
 * 전체 흐름:
 *
 *   1. bp == NULL
 *      malloc(size)와 같은 의미이므로 새 block을 할당한다.
 *
 *   2. size == 0
 *      free(bp)와 같은 의미이므로 기존 block을 해제하고 NULL을 반환한다.
 *
 *   3. 기존 block보다 더 큰 공간이 필요한 경우
 *      - 오른쪽 block이 free이고 두 block의 합이 asize 이상이면
 *        기존 bp를 유지한 채 오른쪽 block을 흡수한다.
 *      - 합친 뒤 16B 이상 남으면 allocated/free block으로 다시 분할한다.
 *      - 오른쪽 block만으로 확장할 수 없으면 새 block을 할당하고,
 *        기존 payload를 복사한 뒤 기존 block을 해제한다.
 *
 *   4. 기존 block이 asize 이상인 경우
 *      - 16B 이상 남으면 뒤쪽을 free block으로 분할하고 coalesce한다.
 *      - 16B보다 적게 남으면 분할하지 않고 기존 block 전체를 유지한다.
 *
 * 현재 구현은 오른쪽 free block을 이용한 제자리 확장까지만 지원한다.
 * 왼쪽 free block 활용, 양쪽 동시 확장, Epilogue 직접 확장은 하지 않는다.
 *
 * 크기를 나타내는 주요 변수:
 *
 *   asize
 *     사용자가 요청한 payload 크기에 Header/Footer와 정렬을 반영한
 *     새로운 block의 전체 크기
 *
 *   old_asize
 *     현재 block의 Header에 기록된 기존 block 전체 크기
 *
 *   next_old_asize
 *     현재 block과 오른쪽 block의 크기를 더한 값
 *     오른쪽 free block을 흡수했을 때 확보할 수 있는 전체 크기
 *
 *   adjust_next_old_asize
 *     오른쪽 free block을 흡수하여 asize만큼 사용한 뒤 남는 크기
 *
 *   temp_size
 *     현재 block을 더 작은 크기로 줄였을 때 남는 크기
 */
void *mm_realloc(void *bp, size_t size)
{
    size_t asize;
    size_t old_asize;
    size_t next_old_asize;
    size_t adjust_next_old_asize;
    size_t temp_size;

    /* realloc(NULL, size)는 malloc(size)와 같다. */
    if (bp == NULL)
        return mm_malloc(size);

    /* realloc(bp, 0)은 기존 block을 해제하고 NULL을 반환한다. */
    if (size == 0) {
        mm_free(bp);
        return NULL;
    }

    /* 요청 payload 크기를 Header/Footer와 정렬을 포함한 block 크기로 변환 */
    asize = ADJUST_SIZE(size);

    /* 현재 allocated block의 전체 크기 */
    old_asize = GET_SIZE(HDRP(bp));

    /* 현재 block과 바로 오른쪽 block을 합쳤을 때의 전체 크기 */
    next_old_asize = old_asize + GET_SIZE(HDRP(NEXT_BLKP(bp)));

    /*
     * 오른쪽 block까지 합친 뒤 요청 크기만큼 사용하고 남는 크기.
     * next_old_asize >= asize인 경우에만 실제 분할 판단에 사용한다.
     */
    adjust_next_old_asize = next_old_asize - asize;

    /* Case 1: 현재 block보다 더 큰 block이 필요한 경우 */
    if (old_asize < asize) {
        /*
         * 오른쪽 block이 free이고 두 block을 합친 크기가 충분하면
         * 기존 bp를 유지한 채 제자리에서 확장한다.
         *
         *        bp                     bp
         *         ↓                      ↓
         *   [현재 ALLOC][오른쪽 FREE] → [확장된 ALLOC][남은 FREE]
         *
         * bp가 바뀌지 않으므로 기존 payload를 따로 복사할 필요가 없다.
         */
        if (!GET_ALLOC(HDRP(NEXT_BLKP(bp))) && next_old_asize >= asize) {
            /*
             * 합친 뒤 16B 이상 남으면 뒤쪽을 free block으로 분할한다.
             * free_bp는 새 allocated block 바로 다음 block의 Payload 시작 주소이다.
             */
            if (adjust_next_old_asize >= 2 * DSIZE) {
                void *free_bp = (char *)bp + asize;

                /* 앞부분을 요청 크기의 allocated block으로 변경 */
                PUT(HDRP(bp), PACK(asize, 1));
                PUT(FTRP(bp), PACK(asize, 1));

                /* 뒤쪽에 남는 공간의 Header/Footer를 free 상태로 기록 */
                PUT(HDRP(bp) + asize, PACK(adjust_next_old_asize, 0));
                PUT(FTRP(free_bp), PACK(adjust_next_old_asize, 0));

                /*
                 * 분할된 free block의 오른쪽도 free일 수 있으므로 coalesce한다.
                 * 이렇게 해야 물리적으로 인접한 free block이 따로 남지 않는다.
                 */
                coalesce(free_bp);
                return bp;
            }

            /*
             * 남는 공간이 16B보다 작으면 독립적인 free block을 만들 수 없다.
             * 따라서 오른쪽 free block 전체를 현재 allocated block에 포함한다.
             */
            PUT(HDRP(bp), PACK(next_old_asize, 1));
            PUT(FTRP(bp), PACK(next_old_asize, 1));
            return bp;
        }

        /*
         * 오른쪽 block만으로 제자리 확장이 불가능한 경우:
         * 새 block을 할당하고 기존 payload를 복사한 뒤 기존 block을 해제한다.
         *
         * 기존 block을 먼저 free하면 그 공간의 데이터가 더 이상 유효하다고
         * 보장할 수 없으므로 반드시 복사를 끝낸 뒤 mm_free(bp)를 호출한다.
         */
        void *new_bp = mm_malloc(size);

        /* 새 block 할당에 실패하면 기존 block은 그대로 유지한다. */
        if (new_bp == NULL)
            return NULL;

        /*
         * old_asize에는 Header와 Footer가 포함되어 있다.
         * DSIZE(8B)를 빼면 기존 block의 최대 payload 크기가 된다.
         * 지금은 더 큰 block으로 확장하는 경우이므로 이 크기만큼 복사해도
         * 새 block의 payload 범위를 넘지 않는다.
         */
        memcpy(new_bp, bp, old_asize - DSIZE);

        /* 복사가 끝난 뒤 기존 block을 free 상태로 변경 */
        mm_free(bp);
        return new_bp;
    }

    /* Case 2: 현재 block이 이미 충분히 큰 경우 */
    temp_size = old_asize - asize;

    if (temp_size >= 2 * DSIZE) {
        /*
         * 뒤쪽에 최소 block 크기(16B) 이상이 남으면 free block으로 분할한다.
         *
         * 먼저 기존 Footer 위치에 temp_size를 기록한다.
         * 현재 Header에는 아직 old_asize가 있으므로 이 시점의 FTRP(bp)는
         * 축소 전 block의 마지막, 즉 새 free block의 Footer 위치를 가리킨다.
         */
        PUT(FTRP(bp), PACK(temp_size, 0));

        /* 앞부분을 축소된 allocated block으로 변경 */
        PUT(HDRP(bp), PACK(asize, 1));
        PUT(FTRP(bp), PACK(asize, 1));

        /* allocated block 바로 뒤에 새 free block의 Header 생성 */
        PUT(HDRP(bp) + asize, PACK(temp_size, 0));

        /*
         * 분할된 free block의 오른쪽도 free라면 즉시 병합한다.
         * 새 free block의 Payload 시작 주소는 bp + asize이다.
         */
        coalesce((char *)bp + asize);
    }
    else {
        /*
         * 남는 공간이 16B보다 작으면 별도 free block으로 만들 수 없다.
         * 내부 단편화를 조금 허용하고 기존 block 크기를 그대로 유지한다.
         */
        PUT(HDRP(bp), PACK(old_asize, 1));
        PUT(FTRP(bp), PACK(old_asize, 1));
    }

    return bp;
}
