#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "rs.h"

#define MAP(x, max_x, m, n) (m + x / (max_x / (n - m) + 1))

double now(time_t epoch)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return ((now.tv_sec - epoch) + now.tv_nsec / 1000000000.0);
}

typedef reed_solomon rs_t;
void cleanup(uint8_t **buf, uint8_t **cmp, uint8_t *marks, int K, int N)
{
    for (int i = 0; i < K + N; i++) {
        free(buf[i]);
        free(cmp[i]);
    }
    free(buf);
    free(cmp);
    free(marks);
}

int run(int seed, int K, int N, int T, double *et, double *dt)
{
    uint8_t **buf = calloc(K + N, T * sizeof(uint8_t *));
    uint8_t **cmp = calloc(K + N, T * sizeof(uint8_t *));
    uint8_t *marks = calloc(1, K + N);
    int ret = 0;

    for (int i = 0; i < K + N; i++) {
        buf[i] = calloc(1, T);
        cmp[i] = calloc(1, T);
    }

    for (int i = 0; i < K; i++) {
        for (int j = 0; j < T; j++) {
            buf[i][j] = MAP(rand(), RAND_MAX, 0, 256);
            cmp[i][j] = buf[i][j];
        }
    }

    reed_solomon_init();
    rs_t *rs = reed_solomon_new(K, N);
    if (!rs) {
        cleanup(buf, cmp, marks, K, N);
        printf("failed to init codec\n");
        return -1;
    }

    double t0 = now(0);
    reed_solomon_encode(rs, buf, K + N, T);
    *et += now(0) - t0;

    for (int i = 0; i < K + N; i++) {
        marks[i] = 0;
    }

    for (int i = 0; i < N; i++) {
        int at = rand() % (K + N);
        memset(buf[at], 0, T);
        marks[at] = 1;
    }

    t0 = now(0);
    ret = reed_solomon_reconstruct(rs, buf, marks, K + N, T);
    *dt += now(0) - t0;
    reed_solomon_release(rs);

    int failed = 0;
    for (int i = 0; i < K; i++) {
        for (int j = 0; j < T; j++) {
            if (cmp[i][j] != buf[i][j]) {
                printf("mismatch at row %d col %d\n", i, j);
                failed = 1;
                break;
            }
        }
    }
    assert(failed == 0);

    cleanup(buf, cmp, marks, K, N);
    return ret;
}

int main(int argc, char *argv[])
{
    double t0 = now(0);
    int seed = time(NULL), K, N, T;
    if (argc != 4)
        return -1;

    K = strtol(argv[1], NULL, 10);
    N = strtol(argv[2], NULL, 10);
    T = strtol(argv[3], NULL, 10);

    srand(seed);

    printf("===BEGIN===P SEED: %d K: %d N: %d T: %d\n", seed, K, N, T);

    int Mb = 128;
    int num = Mb * (1024 * 1024) / (K * T);
    double et = 0.0, dt = 0.0;
    for (int i = 0; i < num; i++) {
        run(seed, K, N, T, &et, &dt);
    }
    printf("data shards = %d, repair shards = %d, encoded %d MB in %1.3f secs, "
           "throughput: %.1fMB/s\n",
           K, N, Mb, et, Mb / et);
    printf("data shards = %d, repair shards = %d, decoded %d MB in %1.3f secs, "
           "throughput: %.1fMB/s\n",
           K, N, Mb, dt, Mb / dt);

    printf("total time: %1.3f|%1.3f|%1.3f\n", now(0) - t0, et, dt);
    return 0;
}
