/*------------------------------------------------------------------------------
* test_util.h : minimal assertion harness (no host-only dependencies, so the
* same tests build and run on the Pico)
*-----------------------------------------------------------------------------*/
#ifndef TEST_UTIL_H
#define TEST_UTIL_H
#include <stdio.h>
#include <math.h>

extern int g_tests_run, g_tests_failed;

#define TEST_BEGIN(name) \
    do { printf("\n== %s ==\n",(name)); } while (0)

#define CHECK(cond,fmt,...) do {                                        \
    g_tests_run++;                                                      \
    if (!(cond)) {                                                      \
        g_tests_failed++;                                               \
        printf("  FAIL %s:%d: " fmt "\n",__FILE__,__LINE__,##__VA_ARGS__); \
    } else {                                                            \
        printf("  ok   " fmt "\n",##__VA_ARGS__);                       \
    }                                                                   \
} while (0)

#define CHECK_NEAR(a,b,tol,fmt,...) do {                                \
    double _a=(a),_b=(b),_t=(tol);                                      \
    g_tests_run++;                                                      \
    if (!(fabs(_a-_b)<=_t)) {                                           \
        g_tests_failed++;                                               \
        printf("  FAIL %s:%d: " fmt " (%.9g vs %.9g, |d|=%.3g > %.3g)\n",  \
               __FILE__,__LINE__,##__VA_ARGS__,_a,_b,fabs(_a-_b),_t);   \
    } else {                                                            \
        printf("  ok   " fmt " (|d|=%.3g)\n",##__VA_ARGS__,fabs(_a-_b));\
    }                                                                   \
} while (0)

#define TEST_SUMMARY() do {                                             \
    printf("\n---------------------------------------------\n");        \
    printf("%d checks, %d failed\n",g_tests_run,g_tests_failed);        \
    printf("%s\n",g_tests_failed?"RESULT: FAIL":"RESULT: PASS");        \
} while (0)

#endif
