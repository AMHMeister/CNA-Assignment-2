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
#define SEQSPACE 14      /* min seq space for SR must be atleast window size * 2, made it 14 */
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

static bool ack_received[SEQSPACE]; /* track packets that have been ACKed */
static bool timer_active[WINDOWSIZE]; /* track active timers */

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

    ack_received[sendpkt.seqnum] = false; /* mark packet as not ACKed */

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer for specific packet */
    starttimer(A,RTT);
    timer_active[windowlast] = true; /* mark timer as active */

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
  int ackcount = 0;
  int i, idx; /* idx to keep track of packet index */
  bool can_slide = true; /* flag to check if window can slide */

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* check if ack is for packet in current window */
    int seqfirst = buffer[windowfirst].seqnum;
    int seqlast = buffer[windowlast].seqnum;

    /* Check if ACK is in current window (handling wrap-around) */
    bool in_window = false;
    if (seqfirst <= seqlast) {
      in_window = (packet.acknum >= seqfirst && packet.acknum <= seqlast);
    } else {
      in_window = (packet.acknum >= seqfirst || packet.acknum <= seqlast);
    }

    if (in_window && !ack_received[packet.acknum]) {
      /* New ACK for a packet in the window */
      if (TRACE > 0)
        printf("----A: ACK %d is for a packet in our window\n", packet.acknum);
      new_ACKs++;
      
      /* Find buffer index of this packet */
      for (i = 0; i < windowcount; i++) {
        idx = (windowfirst + i) % WINDOWSIZE;
        if (buffer[idx].seqnum == packet.acknum) {
          /* Mark packet as acknowledged */
          ack_received[packet.acknum] = true;
          
          /* Stop the timer for this packet */
          if (timer_active[idx]) {
            stoptimer(A);
            timer_active[idx] = false; /* Mark timer as inactive */
          }
          break;
        }
      }
      
      /* Check if we can slide the window */
      while (windowcount > 0 && ack_received[buffer[windowfirst].seqnum]) {
        /* Remove this packet from the window after ack received */
        windowfirst = (windowfirst + 1) % WINDOWSIZE;
        windowcount--;
      }
    }
    else if (TRACE > 0)
      printf("----A: duplicate ACK or ACK outside window received, do nothing!\n");
  }
  else 
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i, idx;
  bool timer_started = false;

  if (TRACE > 0)
    printf("----A: timer interrupt!\n");

  /* Resend only the packet whose timer expired */
  /* Find the packet with expired timer and resend */
  
  /* Attempt to resend the oldest unacknowledged packet and restart its timer */

  for (i = 0; i < windowcount; i++) {
    idx = (windowfirst + i) % WINDOWSIZE;
    if (!ack_received[buffer[idx].seqnum]) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[idx].seqnum);
      
      tolayer3(A, buffer[idx]);
      packets_resent++;
      
      /* Restart timer for this packet */
      starttimer(A, RTT);
      timer_started = true;
      break;
    }
  }
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

  for (int i = 0; i < SEQSPACE; i++) {
    ack_received[i] = false; /* Initialise all ACKs to false */
  }

  for (int i = 0; i < WINDOWSIZE; i++) {
    timer_active[i] = false; /* Initialise all timers to inactive */
  }
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
        int j = 0;
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

