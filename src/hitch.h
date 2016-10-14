/**
  * Copyright 2016 Varnish Software
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  *
  *    1. Redistributions of source code must retain the above
  *       copyright notice, this list of conditions and the following
  *       disclaimer.
  *
  *    2. Redistributions in binary form must reproduce the above
  *       copyright notice, this list of conditions and the following
  *       disclaimer in the documentation and/or other materials
  *       provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY BUMP TECHNOLOGIES, INC. ``AS IS'' AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BUMP
  * TECHNOLOGIES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

#ifndef HITCH_H_INCLUDED
#define HITCH_H_INCLUDED

#include "config.h"

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <limits.h>
#include <syslog.h>

#include <openssl/ssl.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>
#include <openssl/err.h>
#include <openssl/engine.h>
#include <openssl/asn1.h>
#include <openssl/ocsp.h>
#include <openssl/evp.h>
#include <ev.h>

#ifdef __linux__
#include <sys/prctl.h>
#endif

#include "uthash.h"
#include "ringbuffer.h"
#include "miniobj.h"
#include "shctx.h"
#include "vpf.h"
#include "vas.h"
#include "configuration.h"
#include "hssl_locks.h"
#include "asn_gentm.h"
#include "vsb.h"


typedef struct sslstaple_s sslstaple;

struct sni_name_s;
VTAILQ_HEAD(sni_name_head, sni_name_s);

/* SSL contexts. */
struct sslctx_s {
	unsigned		magic;
#define SSLCTX_MAGIC		0xcd1ce5ff
	char			*filename;
	SSL_CTX			*ctx;
	double			mtim;
	sslstaple		*staple;
	int			staple_vfy;
	char			*staple_fn;
	X509			*x509;
	ev_stat			*ev_staple;
	struct sni_name_head	sni_list;
	UT_hash_handle		hh;
};
typedef struct sslctx_s sslctx;

#ifndef OPENSSL_NO_TLSEXT

typedef struct sslstaple_s {
	unsigned	magic;
#define SSLSTAPLE_MAGIC	0x20fe53fd
	unsigned char	*staple;
	double		mtim;
	double		nextupd;
	int		len;
} sslstaple;

/* SNI lookup objects */
typedef struct sni_name_s {
	unsigned		magic;
#define SNI_NAME_MAGIC		0xb0626581
	char			*servername;
	sslctx			*sctx;
	int			is_wildcard;
	VTAILQ_ENTRY(sni_name_s)	list;
	UT_hash_handle		hh;
} sni_name;

sni_name *sni_names;

#endif /* OPENSSL_NO_TLSEXT */

/*
 * Proxied State
 *
 * All state associated with one proxied connection
 */
typedef struct proxystate {
	unsigned		magic;
#define PROXYSTATE_MAGIC	0xcf877ed9
	ringbuffer		ring_ssl2clear;	/* Pushing bytes from
						 * secure to clear
						 * stream */
	ringbuffer		ring_clear2ssl;	/* Pushing bytes from
						 * clear to secure
						 * stream */
	ev_io 			ev_r_ssl;	/* Secure stream write event */
	ev_io			ev_w_ssl;	/* Secure stream read event */
	ev_io			ev_r_handshake;	/* Secure stream handshake
						 * write event */
	ev_io			ev_w_handshake;	/* Secure stream handshake
						 * read event */
	ev_timer		ev_t_handshake;	/* handshake timer */
	ev_io			ev_w_connect;	/* Backend connect event */
	ev_timer		ev_t_connect;	/* backend connect timer */

	ev_io			ev_r_clear;	/* Clear stream write event */
	ev_io			ev_w_clear;	/* Clear stream read event */
	ev_io			ev_proxy;	/* proxy read event */

	int			fd_up;		/* Upstream (client) socket */
	int			fd_down;	/* Downstream (backend)
						 * socket */

	int			want_shutdown:1; /* Connection is
						  * half-shutdown */
	int			handshaked:1;	/* Initial handshake happened */
	int			clear_connected:1; /* Clear stream is
						    * connected */
	int			renegotiation:1; /* Renegotation is
						  * occuring */
	int			npn_alpn_tried:1;/* NPN or ALPN was tried */

	SSL			*ssl;		/* OpenSSL SSL state */

	struct sockaddr_storage	remote_ip;	/* Remote ip returned
						 * from `accept` */
	int			connect_port;	/* local port for connection */
} proxystate;


/* declare printf like functions: */
void WLOG(int level, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));
void logproxy(int level, const proxystate* ps, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));

void VWLOG(int level, const char *fmt, va_list ap);
void WLOG(int level, const char *fmt, ...);

#define LOG(...)							\
	do {								\
		if (!CONFIG->QUIET)					\
			WLOG(LOG_INFO, __VA_ARGS__ );			\
	} while (0)
#define ERR(...)	WLOG(LOG_ERR, __VA_ARGS__ )

#define LOGL(...) WLOG(LOG_INFO, __VA_ARGS__)

#define SOCKERR(msg)						\
	do {							\
		if (errno == ECONNRESET) {			\
			LOG(msg ": %s\n", strerror(errno));	\
		} else {					\
			ERR(msg ": %s\n", strerror(errno));	\
		}						\
	} while (0)

void log_ssl_error(proxystate *ps, const char *what, ...);

// void logproxy(int level, const proxystate* ps, const char *fmt, ...);

#define LOGPROXY(...)							\
	do {								\
		if (!CONFIG->QUIET && (logf || CONFIG->SYSLOG))		\
			logproxy(LOG_INFO, __VA_ARGS__ );		\
	} while(0)

#define ERRPROXY(...)							\
	do {								\
		if (logf || CONFIG->SYSLOG)				\
			logproxy(LOG_ERR, __VA_ARGS__ );		\
	} while (0)

// hitch_config *CONFIG;
// struct ev_loop *loop;

double Time_now(void);

X509 * Find_issuer(X509 *subj, STACK_OF(X509) *chain);

#endif  /* HITCH_H_INCLUDED */