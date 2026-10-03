/*------------------------------------------------------------------------------
* lockloss.h : count receiver-wide carrier-lock losses
*
* A satellite losing carrier lock is normal: it sets, it is blocked, its
* signal fades. Every satellite losing it in the same epoch while C/N0 holds
* steady is not -- the sky did not change, the receiver did. The 2026-09-21
* field capture had one such event every ~25 s, each of which reset every
* ambiguity in RTKLIB and in the receiver's own RTK engine alike. This counts
* them from the decoded observations, so the bench firmware can show the rate
* live and the host replay can count it in a capture, with the same rule.
*-----------------------------------------------------------------------------*/
#ifndef LOCKLOSS_H
#define LOCKLOSS_H

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* With fewer satellites tracked, a receiver-wide event cannot be told apart
 * from a poor sky view. */
#define LOCKLOSS_MIN_SATS  6
/* Fraction of tracked satellites that must slip in one epoch. Real sky events
 * (a tree, a banking airframe) take out a sector, not the whole sky. */
#define LOCKLOSS_FRACTION  0.6
/* Loss epochs closer together than this are one event: the receiver often
 * drops, re-acquires for an epoch or two, and drops again. */
#define LOCKLOSS_MERGE_S   1.0

typedef struct {
    uint32_t epochs;        /* rover epochs examined                          */
    uint32_t events;        /* receiver-wide lock losses                      */
    gtime_t  first;         /* first epoch examined                           */
    gtime_t  latest;        /* latest epoch examined                          */
    gtime_t  last_loss;     /* latest epoch that was part of an event         */
    int      last_nslip;    /* satellites that slipped in that epoch ...      */
    int      last_ntrk;     /* ... out of this many tracked                   */
} lockloss_t;

void lockloss_init(lockloss_t *l);

/* Examine one decoded rover epoch. Returns 1 if it starts a new event. */
int lockloss_epoch(lockloss_t *l, const obsd_t *obs, int n);

/* Minutes spanned by the epochs examined so far. */
double lockloss_minutes(const lockloss_t *l);

#ifdef __cplusplus
}
#endif
#endif /* LOCKLOSS_H */
