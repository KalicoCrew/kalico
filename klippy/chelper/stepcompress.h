#ifndef STEPCOMPRESS_H
#define STEPCOMPRESS_H

#include <stdint.h> // uint32_t

#define ERROR_RET -989898989

struct stepcompress *stepcompress_alloc(uint32_t oid);
void stepcompress_free(struct stepcompress *sc);
int stepcompress_queue_mq_msg(struct stepcompress *sc, uint64_t req_clock
                              , uint32_t *data, int len);

#endif // stepcompress.h
