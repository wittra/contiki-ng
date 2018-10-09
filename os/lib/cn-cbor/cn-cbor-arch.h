/*
 * cn-cbor-arch.h
 *
 *  Created on: Oct 9, 2018
 *      Author: adrian
 */

#ifndef OS_LIB_CN_CBOR_CN_CBOR_ARCH_H_
#define OS_LIB_CN_CBOR_CN_CBOR_ARCH_H_

#include "contiki.h"
#include "contiki-net.h"
#include "heapmem.h"

#ifndef HEAPMEM_CONF_ARENA_SIZE
#define HEAPMEM_CONF_ARENA_SIZE     512
#endif
#define CN_CALLOC   heapmem_alloc(sizeof(cn_cbor))
#define CN_FREE     heapmem_free

#endif /* OS_LIB_CN_CBOR_CN_CBOR_ARCH_H_ */
