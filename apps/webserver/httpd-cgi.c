/*
 * Copyright (c) 2001, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: httpd-cgi.c,v 1.16 2010/10/19 18:29:03 adamdunkels Exp $
 *
 */

/*
 * This file includes functions that are called by the web server
 * scripts. The functions takes no argument, and the return value is
 * interpreted as follows. A zero means that the function did not
 * complete and should be invoked for the next packet as well. A
 * non-zero value indicates that the function has completed and that
 * the web server should move along to the next script line.
 *
 */

#include <stdio.h>
#include <string.h>

#include "contiki-net.h"
#include "httpd.h"
#include "httpd-cgi.h"
#include "httpd-fs.h"

#include "lib/petsciiconv.h"

static struct httpd_cgi_call *calls = NULL;

static const char closed[] =   /*  "CLOSED",*/
{0x43, 0x4c, 0x4f, 0x53, 0x45, 0x44, 0};
static const char syn_rcvd[] = /*  "SYN-RCVD",*/
{0x53, 0x59, 0x4e, 0x2d, 0x52, 0x43, 0x56,
 0x44,  0};
static const char syn_sent[] = /*  "SYN-SENT",*/
{0x53, 0x59, 0x4e, 0x2d, 0x53, 0x45, 0x4e,
 0x54,  0};
static const char established[] = /*  "ESTABLISHED",*/
{0x45, 0x53, 0x54, 0x41, 0x42, 0x4c, 0x49,
 0x53, 0x48, 0x45, 0x44, 0};
static const char fin_wait_1[] = /*  "FIN-WAIT-1",*/
{0x46, 0x49, 0x4e, 0x2d, 0x57, 0x41, 0x49,
 0x54, 0x2d, 0x31, 0};
static const char fin_wait_2[] = /*  "FIN-WAIT-2",*/
{0x46, 0x49, 0x4e, 0x2d, 0x57, 0x41, 0x49,
 0x54, 0x2d, 0x32, 0};
static const char closing[] = /*  "CLOSING",*/
{0x43, 0x4c, 0x4f, 0x53, 0x49,
 0x4e, 0x47, 0};
static const char time_wait[] = /*  "TIME-WAIT,"*/
{0x54, 0x49, 0x4d, 0x45, 0x2d, 0x57, 0x41,
 0x49, 0x54, 0};
static const char last_ack[] = /*  "LAST-ACK"*/
{0x4c, 0x41, 0x53, 0x54, 0x2d, 0x41, 0x43,
 0x4b, 0};
static const char none[] = /*  "NONE"*/
{0x4e, 0x4f, 0x4e, 0x45, 0};
static const char running[] = /*  "RUNNING"*/
{0x52, 0x55, 0x4e, 0x4e, 0x49, 0x4e, 0x47,
 0};
static const char called[] = /*  "CALLED"*/
{0x43, 0x41, 0x4c, 0x4c, 0x45, 0x44, 0};
static const char file_name[] = /*  "file-stats"*/
{0x66, 0x69, 0x6c, 0x65, 0x2d, 0x73, 0x74,
 0x61, 0x74, 0x73, 0};
static const char tcp_name[] = /*  "tcp-connections"*/
{0x74, 0x63, 0x70, 0x2d, 0x63, 0x6f, 0x6e,
 0x6e, 0x65, 0x63, 0x74, 0x69, 0x6f, 0x6e,
 0x73, 0};
static const char proc_name[] = /*  "processes"*/
{0x70, 0x72, 0x6f, 0x63, 0x65, 0x73, 0x73,
 0x65, 0x73, 0};
#if HTTPD_CONF_PASS_QUERY_STRING
static const char tictactoe_name[] = /*  "tictactoe"*/
{0x74, 0x69, 0x63, 0x74, 0x61, 0x63, 0x74,
 0x6f, 0x65, 0};
#endif

static const char *states[] = {
  closed,
  syn_rcvd,
  syn_sent,
  established,
  fin_wait_1,
  fin_wait_2,
  closing,
  time_wait,
  last_ack,
  none,
  running,
  called};

/*---------------------------------------------------------------------------*/
static
PT_THREAD(nullfunction(struct httpd_state *s, char *ptr))
{
  PSOCK_BEGIN(&s->sout);
  PSOCK_END(&s->sout);
}
/*---------------------------------------------------------------------------*/
httpd_cgifunction
httpd_cgi(char *name)
{
  struct httpd_cgi_call *f;

  /* Find the matching name in the table, return the function. */
  for(f = calls; f != NULL; f = f->next) {
    if(strncmp(f->name, name, strlen(f->name)) == 0) {
      return f->function;
    }
  }
  return nullfunction;
}
/*---------------------------------------------------------------------------*/
static unsigned short
generate_file_stats(void *arg)
{
  char *f = (char *)arg;
  return snprintf((char *)uip_appdata, uip_mss(), "%5u", httpd_fs_count(f));
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(file_stats(struct httpd_state *s, char *ptr))
{
  PSOCK_BEGIN(&s->sout);

  PSOCK_GENERATOR_SEND(&s->sout, generate_file_stats, (void *) (strchr(ptr, ' ') + 1));
  
  PSOCK_END(&s->sout);
}
/*---------------------------------------------------------------------------*/
static unsigned short
make_tcp_stats(void *arg)
{
  struct uip_conn *conn;
  struct httpd_state *s = (struct httpd_state *)arg;
  conn = &uip_conns[s->u.count];

  #if UIP_CONF_IPV6
  char buf[48];
  httpd_sprint_ip6(conn->ripaddr, buf);
  return snprintf((char *)uip_appdata, uip_mss(),
         "<tr align=\"center\"><td>%d</td><td>%s:%u</td><td>%s</td><td>%u</td><td>%u</td><td>%c %c</td></tr>\r\n",
         uip_htons(conn->lport),
         buf,
         uip_htons(conn->rport),
         states[conn->tcpstateflags & UIP_TS_MASK],
         conn->nrtx,
         conn->timer,
         (uip_outstanding(conn))? '*':' ',
         (uip_stopped(conn))? '!':' ');
#else
  return snprintf((char *)uip_appdata, uip_mss(),
         "<tr align=\"center\"><td>%d</td><td>%u.%u.%u.%u:%u</td><td>%s</td><td>%u</td><td>%u</td><td>%c %c</td></tr>\r\n",
         uip_htons(conn->lport),
         conn->ripaddr.u8[0],
         conn->ripaddr.u8[1],
         conn->ripaddr.u8[2],
         conn->ripaddr.u8[3],
         uip_htons(conn->rport),
         states[conn->tcpstateflags & UIP_TS_MASK],
         conn->nrtx,
         conn->timer,
        (uip_outstanding(conn))? '*':' ',
        (uip_stopped(conn))? '!':' ');
#endif /* UIP_CONF_IPV6 */
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(tcp_stats(struct httpd_state *s, char *ptr))
{
  
  PSOCK_BEGIN(&s->sout);

  for(s->u.count = 0; s->u.count < UIP_CONNS; ++s->u.count) {
    if((uip_conns[s->u.count].tcpstateflags & UIP_TS_MASK) != UIP_CLOSED) {
      PSOCK_GENERATOR_SEND(&s->sout, make_tcp_stats, s);
    }
  }

  PSOCK_END(&s->sout);
}
/*---------------------------------------------------------------------------*/
static unsigned short
make_processes(void *p)
{
  char name[40];

  strncpy(name, ((struct process *)p)->name, 40);
  petsciiconv_toascii(name, 40);

  return snprintf((char *)uip_appdata, uip_mss(),
		 "<tr align=\"center\"><td>%p</td><td>%s</td><td>%p</td><td>%s</td></tr>\r\n",
		 p, name,
		 *((char **)&(((struct process *)p)->thread)),
		 states[9 + ((struct process *)p)->state]);
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(processes(struct httpd_state *s, char *ptr))
{
  PSOCK_BEGIN(&s->sout);
  for(s->u.ptr = PROCESS_LIST(); s->u.ptr != NULL; s->u.ptr = ((struct process *)s->u.ptr)->next) {
    PSOCK_GENERATOR_SEND(&s->sout, make_processes, s->u.ptr);
  }
  PSOCK_END(&s->sout);
}
#if HTTPD_CONF_PASS_QUERY_STRING
extern char httpd_query[HTTPD_CONF_PASS_QUERY_STRING];
/*---------------------------------------------------------------------------*/
static uint8_t whowon(char x) {

  if (httpd_query[0]==x) {
    if(((httpd_query[1]==x)&&(httpd_query[2]==x))
    || ((httpd_query[4]==x)&&(httpd_query[8]==x))
    || ((httpd_query[3]==x)&&(httpd_query[6]==x))){
      return 1;
    }
  } else if (httpd_query[1]==x) {
    if ((httpd_query[4]==x)&&(httpd_query[7]==x)) {
      return 1;
    }
  } else if (httpd_query[2]==x) {
    if(((httpd_query[4]==x)&&(httpd_query[6]==x))
    || ((httpd_query[5]==x)&&(httpd_query[8]==x))){
      return 1;
    }
  } else if (httpd_query[3]==x) {
    if ((httpd_query[4]==x)&&(httpd_query[5]==x)) {
      return 1;
    }
  } else if (httpd_query[6]==x) {
    if ((httpd_query[7]==x)&&(httpd_query[8]==x)) {
      return 1;
    }
  }
 return 0;
}
/*---------------------------------------------------------------------------*/
static unsigned short
make_tictactoe(void *p)
{
  uint8_t i,newgame,iwon,uwon,nx,no;
  char me,you,locater;
  unsigned short numprinted=0;
 
 /* If no query string restart game, else put into proper form */
  newgame=0;httpd_query[9]=0;
  if ((httpd_query[0]==0)||(httpd_query[0]==' ')) {
    newgame=1;
    for (i=0;i<9;i++) httpd_query[i]='b';
  } else for (i=0;i<9;i++) {
    if (!((httpd_query[i]=='x')||(httpd_query[i]=='o'))) {
      httpd_query[i]='b';
    }
  }

  /* I am x if I move first, or if number of x's is <= number of o's */
  for (nx=0,no=0,i=0;i<9;i++) {
    if (httpd_query[i]=='x') nx++;
    else if (httpd_query[i]=='o') no++;
  }
  if ((no>=nx)&&!newgame) {me='x';you='o';}
  else {me='o';you='x';};

  iwon=whowon(me);
  uwon=whowon(you);

  if (newgame||iwon||uwon||(nx+no)>=9) goto showboard;
 
  /* Make a move */
  if (me=='x') nx++;else no++;
  if (httpd_query[4]=='b') httpd_query[4]=me;
  else if (httpd_query[0]=='b') httpd_query[0]=me;
  else if (httpd_query[2]=='b') httpd_query[2]=me;
  else if (httpd_query[6]=='b') httpd_query[6]=me;
  else if (httpd_query[8]=='b') httpd_query[8]=me;
   else if (httpd_query[1]=='b') httpd_query[1]=me;
  else if (httpd_query[3]=='b') httpd_query[3]=me;
  else if (httpd_query[5]=='b') httpd_query[5]=me;
  else if (httpd_query[7]=='b') httpd_query[7]=me;
  
  /* Did I win? */
  iwon=whowon(me);

  showboard: 
  for (i=0;i<9;i++) {
  
    if (i==4) locater='c';
    else if ((i==1)||(i==7)) locater='v';
    else if ((i==3)||(i==5)) locater='h';
    else locater=0;
    
    if ((httpd_query[i]=='b')&&(!(iwon||uwon))) {
        httpd_query[i]=you;
        numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<a href=ttt.shtml?%s><img src=b",httpd_query);
        httpd_query[i]='b';
    } else {
        numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<img src=%c",httpd_query[i]);
    }
    if (locater) {
        numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "%c",locater);
    }
    numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, ".gif>");
    if (httpd_query[i]=='b') {       
        numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "</a>");       
    }
    if ((i==2)||(i==5)) {
      numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br>");
    }
  }
  
  if ((nx>(no+1))||(no>(nx+1))) {
     numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br><h2>You cheated!!!</h2>");
  } else if (iwon) {
     numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br><h2>I Win!</h2>");
  } else if (uwon) { 
     numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br><h2>You Win!</h2>");
  } else if ((nx+no)==9) {
     numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br><h2>Draw!</h2>");
  }
  if (iwon||uwon||((nx+no)==9)) {
       numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br><a href=ttt.shtml>Play Again</a>");
  }

  /* If new game give option for me to start */
  if ((nx==0)&&(no==0)) {
     numprinted+=snprintf((char *)uip_appdata+numprinted, uip_mss()-numprinted, "<br><br><a href=ttt.shtml?bbbbbbbb>Let computer move first</a>");
  }
  httpd_query[0]=0;  //zero the query string
  printf("numprinted=%d\n",numprinted);
  return numprinted;

}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(tictactoe(struct httpd_state *s, char *ptr))
{
  PSOCK_BEGIN(&s->sout);
  PSOCK_GENERATOR_SEND(&s->sout, make_tictactoe, s);
  PSOCK_END(&s->sout);
}
#endif
/*---------------------------------------------------------------------------*/
void
httpd_cgi_add(struct httpd_cgi_call *c)
{
  struct httpd_cgi_call *l;

  c->next = NULL;
  if(calls == NULL) {
    calls = c;
  } else {
    for(l = calls; l->next != NULL; l = l->next);
    l->next = c;
  }
}
/*---------------------------------------------------------------------------*/

HTTPD_CGI_CALL(file, file_name, file_stats);
HTTPD_CGI_CALL(tcp, tcp_name, tcp_stats);
HTTPD_CGI_CALL(proc, proc_name, processes);
#if HTTPD_CONF_PASS_QUERY_STRING
HTTPD_CGI_CALL(tictac, tictactoe_name, tictactoe);
#endif

void
httpd_cgi_init(void)
{
  httpd_cgi_add(&file);
  httpd_cgi_add(&tcp);
  httpd_cgi_add(&proc);
#if HTTPD_CONF_PASS_QUERY_STRING
  httpd_cgi_add(&tictac);
#endif
}
/*---------------------------------------------------------------------------*/
