#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* gcc -Wall -ansi -pedantic -o sr emulator.c sr.c */

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
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
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
  /* if blocked,  window is full */
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
  windowcount = 0;
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */

static struct pkt recv_buffer[WINDOWSIZE]; /* buffer for packets that are out of order */
static int recv_base;                      /* base seq num of receive window */
static bool packet_received[SEQSPACE];     /* track packets that have been received */
static int recv_windowsize;                /* size of the receive window */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int rel_seqnum; /* Relative sequence number in the receive window */
  int j = 0;

  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----B: packet %d correctly received\n", packet.seqnum);
    
    /* Calculate relative sequence number ( aka distance from recv_base) */
    if (packet.seqnum >= recv_base)
      rel_seqnum = packet.seqnum - recv_base;
    else
      rel_seqnum = SEQSPACE - recv_base + packet.seqnum;
    
    /* Check if packet is within receive window */
    if (rel_seqnum < recv_windowsize) {
      /* Mark packet as received if condition is met*/
      packet_received[packet.seqnum] = true;
      
      /* Store packet in buffer */
      recv_buffer[rel_seqnum] = packet;
      
      /* If the packet is what we're expecting next, deliver it and any consecutive buffered packets */
      if (packet.seqnum == recv_base) {
        /* Deliver packet */
        tolayer5(B, packet.payload);
        packets_received++;
        
        /* Update recv_base and deliver any consecutive buffered packets */
        
        do {
          recv_base = (recv_base + 1) % SEQSPACE;
          j++;
          
          /* Check if next packet is already buffered */
          if (packet_received[recv_base]) {
            /* Deliver packet */
            tolayer5(B, recv_buffer[(j % recv_windowsize)].payload);
            packets_received++;
            packet_received[recv_base] = false; /* Reset flag for later */
          } else {
            break; /* No more consecutive packets */
          }
        } while (true);
      }
    }
    
    /* Send ACK for the packet */
    sendpkt.acknum = packet.seqnum;
  }
  
  else {
    /* Packet is corrupted, do not ACK */
    if (TRACE > 0) 
      printf("----B: packet corrupted, no ACK sent\n");
    return; /* No NAKs in SR */
  }

  /* Create ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* No data to send. Fill payload with 0's */
  for (i = 0; i < 20; i++) 
    sendpkt.payload[i] = '0';  

  /* Compute checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* Send packet */
  if (TRACE > 0)
    printf("----B: sending ACK %d\n", sendpkt.acknum);
  tolayer3(B, sendpkt);
}

void B_init(void)
{
  int i;

  expectedseqnum = 0; 
  B_nextseqnum = 0;  

  recv_base = 0;      
  recv_windowsize = WINDOWSIZE; 

  for (i = 0; i < SEQSPACE; i++) {
    packet_received[i] = false; /* Initialise all packets to not received */
  }

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

