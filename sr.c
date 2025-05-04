#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* Compile Command: gcc -Wall -ansi -pedantic -o sr emulator.c sr.c */

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12      /* min seq space for SR must be atleast window size * 2 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

static int ACKed[WINDOWSIZE];            /* array for storing acked packets */

/* called from layer 5 (application layer), passed the message to be sent to other side */


void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    windowcount++;

    ACKed[windowlast] = 0; /* sign non acked packets with 0 */

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer for specific packet */
    if (windowcount == 1) 
      starttimer(A, RTT); 
      
    

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int ack_count = 0;
  int seq_count;
  int expected_ACK;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* mark pkt as received */

    /* Advance window base to next unacked if the packet is the first one in the window*/
    expected_ACK = A_nextseqnum - windowcount;
    if (expected_ACK < 0) {
      expected_ACK = expected_ACK + SEQSPACE;
    }

    seq_count = packet.acknum - expected_ACK;
    if (seq_count < 0) {
      seq_count = seq_count + SEQSPACE;
    }

    /* check if ack is new or a duplicate */
    if ((windowcount > 0) && (seq_count < WINDOWSIZE)) {

      /* if packet is a new ack */
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n",packet.acknum);
      new_ACKs++;

      ack_count = (seq_count + windowfirst) % WINDOWSIZE;
      ACKed[ack_count] = NOTINUSE;

      /* slide window */
      while ((ACKed[windowfirst] == NOTINUSE) && (windowcount >0)) {

    	/* slide window by number of acked packets  */
        windowfirst = (windowfirst+1) % WINDOWSIZE;

      /* delete acked packets from the window buffer */
    	windowcount--;
      }

      /* start timer again if there are more unacked packets in the window */
      if (seq_count == 0) {
        stoptimer(A);
    	if (windowcount > 0)
    	  starttimer (A,RTT);
      }

    } else
    	if (TRACE > 0)
    	  printf ("----A:duplicate ACK received, do nothing!\n");
  } else
	  if (TRACE>0)
	    printf ("----A:corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  if (TRACE > 0)
    printf ("---A: resending packet %d\n", (buffer[windowfirst]).seqnum);

  /* only resend the oldest packet in window*/
  tolayer3 (A,buffer[windowfirst]);
  packets_resent++;
  if (windowcount > 0)
      starttimer(A,RTT);
}       


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0; /* Initialise window count */
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */

static struct pkt recv_buffer[WINDOWSIZE]; /* buffer for packets that are out of order */
static int B_windowfirst;           /* the index of the first packet in B_buffer */
int i;



/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt send_pkt;
  int i;
  int seq_count;

  /* if uncorrupted */
  if  ( (!IsCorrupted(packet)) ) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    packets_received++;
    
    /* buffer out-of-order packets */
    /* deliver ACKs in any order, regardless of buffered status */
    /* advance window to next unreceived packet */

    seq_count = packet.seqnum - expectedseqnum;
    if (seq_count < 0)
      seq_count = seq_count + SEQSPACE;

    if (seq_count < WINDOWSIZE) {
      recv_buffer[((B_windowfirst + seq_count) % WINDOWSIZE)] = packet;
      while (recv_buffer[B_windowfirst].seqnum != NOTINUSE) {

    	/* deliver to application */
    	tolayer5(B, recv_buffer[B_windowfirst].payload);

    	recv_buffer[B_windowfirst].seqnum = NOTINUSE;
    	B_windowfirst = (B_windowfirst + 1) % WINDOWSIZE;

    	/* update state variables */
    	expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
      }
    }
    /* create packet */
    /* send an ACK for received packet */

    send_pkt.acknum = packet.seqnum;

    send_pkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    /* fill payload with 0's if there is no data to send */
    for ( i=0; i<20 ; i++ )
      send_pkt.payload[i] = '0';

    /* compute checksum */
    send_pkt.checksum = ComputeChecksum(send_pkt);

    /* send out packet */
    tolayer3 (B, send_pkt);
  }
  
}

void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;

  
  for ( i=0;i < WINDOWSIZE; i++) {
	  recv_buffer[i].seqnum = NOTINUSE;
  }
  B_windowfirst = 0; /* initialise index of first packet */
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

