
/*****************************************************************
 *
 *      Author :                     Ned D. Arnold
 *      Date:                        11/21/97
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *****************************************************************
 *                         COPYRIGHT NOTIFICATION
 *****************************************************************

 * THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
 * AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
 * AND IN ALL SOURCE LISTINGS OF THE CODE.
 
 * (C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO
 
 * Argonne National Laboratory (ANL), with facilities in the States of 
 * Illinois and Idaho, is owned by the United States Government, and
 * operated by the University of Chicago under provision of a contract
 * with the Department of Energy.

 * Portions of this material resulted from work developed under a U.S.
 * Government contract and are subject to the following license:  For
 * a period of five years from March 30, 1993, the Government is
 * granted for itself and others acting on its behalf a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, and perform
 * publicly and display publicly.  With the approval of DOE, this
 * period may be renewed for two additional five year periods. 
 * Following the expiration of this period or periods, the Government
 * is granted for itself and others acting on its behalf, a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, distribute copies
 * to the public, perform publicly and display publicly, and to permit
 * others to do so.

 *****************************************************************
 *                               DISCLAIMER
 *****************************************************************

 * NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
 * THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
 * MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
 * LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
 * USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
 * DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
 * OWNED RIGHTS.  

 *****************************************************************
 * LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
 * DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
 *****************************************************************

 * Modification Log:
 * -----------------
 * 01-23-98   nda       initially functional
 * 10-02-98   nda       functional
 * 10-06-98   nda       fixed a bug with li,lo,ai,ao where sum of bit+
 *                      numbits > MAX_ACTIVE_BITS
 * 01-14-09   ses       Modified for R3.14/OSI              
 */

/*To Use this device support, Include the following before iocInit */
/* devA16VmeConfig(card,a16base,nreg,iVector,iLevel,iReg)  */
/*    card    = card number                                */
/*    a16base = base address of card                       */
/*    nreg    = number of A16 registers on this card       */
/*    iVector = interrupt vector                           */
/*    iLevel  = interrupt level                            */
/*    iRegister = register # for interrupt vector          */
/* For Example					           */
/* devA16VmeConfig(0, 0x8000, 12, 0, 0, 7)                 */


 /**********************************************************************/
 /** Brief Description of device support                              **/
 /**						    	              **/
 /** This device support allows access to any register of a VME       **/
 /** module found in the A16/D16 VME space. The bit field of interest **/
 /** is described in the PARM field of the INP or OUT link.           **/
 /** This allows a generic driver to be used without hard-coding      **/
 /** register numbers within the software.                            **/
 /**						    	              **/
 /** Record type     Signal #           Parm Field                    **/
 /**				   reg_offset, lsb, width             **/
 /**                                                                  **/
 /**    ai          reg_offset     lsb, width                         **/
 /**    ao          reg_offset     lsb, width                         **/
 /**    bi          reg_offset     bit #                              **/
 /**    bo          reg_offset     bit #                              **/
 /**    longin      reg_offset     lsb, width                         **/
 /**    longout     reg_offset     lsb, width                         **/
 /**    mbbi        reg_offset     lsb                                **/
 /**    mbbo        reg_offset     lsb                                **/
 /**                                                                  **/
 /** reg_offset is specified by the register number (0,1,2,3, etc)    **/
 /** Parm field must be provided, no defaults are assumed ...         **/
 /**                                                                  **/
 /**                                                                  **/
 /**                                                                  **/
 /**********************************************************************/

#include	<string.h>
#include	<math.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include        <recGbl.h>
#include	<devSup.h>
#include	<link.h>
#include	<dbScan.h>

#include	<dbCommon.h>
#include        <aoRecord.h>
#include        <aiRecord.h>
#include        <boRecord.h>
#include        <biRecord.h>
#include        <longinRecord.h>
#include        <longoutRecord.h>
#include        <mbboRecord.h>
#include        <mbbiRecord.h>

#include	<epicsExport.h>
#include	<epicsInterrupt.h>
#include	<epicsStdlib.h>
#include	<epicsStdioRedirect.h>
#include	<epicsPrint.h>
#include	<epicsExit.h>
#include	<devLib.h>
#include	<iocsh.h>


static long init_ai(struct aiRecord *pai);
static long read_ai(struct aiRecord *pai);
static long init_ao(struct aoRecord *pao);
static long write_ao(struct aoRecord *pao);
static long init_bi(struct biRecord *pbi);
static long read_bi(struct biRecord *pbi);
static long init_bo(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);
static long init_li(struct longinRecord *pli);
static long read_li(struct longinRecord *pli);
static long init_lo(struct longoutRecord *plo);
static long write_lo(struct longoutRecord *plo);
static long init_mbbi(struct mbbiRecord *pmbbi);
static long read_mbbi(struct mbbiRecord *pmbbi);
static long init_mbbo(struct mbboRecord *pmbbo);
static long write_mbbo(struct mbboRecord *pmbbo);
static long checkCard(short card);
static long write_card(short card, epicsUInt16 reg, epicsUInt32 mask, epicsUInt32 value); 
static long read_card(short card, epicsUInt16 reg, epicsUInt32 mask, epicsUInt32 *value);
static long get_bi_int_info(int cmd, struct biRecord *pbi, IOSCANPVT *ppvt);
static void devA16_isr(void *arg);
static void devA16RebootFunc(void *arg);
static long devA16VmeReport();

int  devA16VmeDebug = 0; 

/***** devA16VmeDebug *****/

/** devA16VmeDebug == 0 --- no debugging messages **/
/** devA16VmeDebug >= 5 --- hardware initialization information **/
/** devA16VmeDebug >= 10 -- record initialization information **/
/** devA16VmeDebug >= 15 -- write commands **/
/** devA16VmeDebug >= 20 -- read commands **/


#define MAX_ACTIVE_BITS  16   /* largest bit # expected */

#define IVEC_MASK          0xff  /* Interrupt Vector Mask      */
#define IVEC_ENABLE_MASK   0x100 /* Interrupt Enable Mask      */

/* Register layout */
typedef struct a16Reg {
  volatile epicsUInt16  reg[1];
}a16Reg;

typedef struct ioCard {  /* Unique for each card */
  volatile a16Reg  *base;    /* address of this card's registers */
  int               nReg;    /* Number of registers on this card */
  int               iReg;    /* Interrupt Register # */
  IOSCANPVT         ioscanpvt; /* records to process upon interrupt */
}ioCard;

static struct ioCard *cards; /* array of card info */
static int num_cards;

typedef struct a16VmeDpvt { /* unique for each record */
  epicsUInt16  reg;   /* index of register to use (determined by signal #*/
  epicsUInt16  lbit;  /* least significant bit of interest */
  epicsUInt32  mask;  /* mask to use ...  */
}a16VmeDpvt;

/* Define the dset for A16VME */
typedef struct {
	long		number;
	DEVSUPFUN	report;		/* used by dbior */
	DEVSUPFUN	init;	
	DEVSUPFUN	init_record;	/* called 1 time for each record */
	DEVSUPFUN	get_ioint_info;	
	DEVSUPFUN	read_write;
        DEVSUPFUN       special_linconv;
} A16VME_DSET;

A16VME_DSET devAiA16Vme =   {6, NULL, NULL, init_ai, NULL, read_ai,  NULL};
A16VME_DSET devAoA16Vme =   {6, NULL, NULL, init_ao, NULL, write_ao, NULL};
A16VME_DSET devBiA16Vme =   {5, devA16VmeReport,NULL,init_bi, get_bi_int_info, 
                             read_bi,  NULL};
A16VME_DSET devBoA16Vme =   {5, NULL, NULL, init_bo, NULL, write_bo, NULL};
A16VME_DSET devLiA16Vme =   {5, NULL, NULL, init_li, NULL, read_li,  NULL};
A16VME_DSET devLoA16Vme =   {5, NULL, NULL, init_lo, NULL, write_lo, NULL};
A16VME_DSET devMbbiA16Vme = {5, NULL, NULL, init_mbbi, NULL, read_mbbi,  NULL};
A16VME_DSET devMbboA16Vme = {5, NULL, NULL, init_mbbo, NULL, write_mbbo, NULL};

epicsExportAddress(dset, devAiA16Vme);
epicsExportAddress(dset, devAoA16Vme);
epicsExportAddress(dset, devBiA16Vme);
epicsExportAddress(dset, devBoA16Vme);
epicsExportAddress(dset, devLiA16Vme);
epicsExportAddress(dset, devLoA16Vme);
epicsExportAddress(dset, devMbbiA16Vme);
epicsExportAddress(dset, devMbboA16Vme);

/**************************************************************************
 **************************************************************************/
static long devA16VmeReport()
{
int             i;
int		cardNum = 0;
epicsUInt32     regData;

  for(cardNum=0; cardNum < num_cards; cardNum++) {
    if(cards[cardNum].base != NULL) {
      printf("  Card #%d at %p\n", cardNum, cards[cardNum].base);
      for(i=0; i < cards[cardNum].nReg; i++) {
          regData = cards[cardNum].base->reg[i];
          printf("    Register %d -> 0x%4.4X (%d)\n", i, regData, regData);
      }
    }
  }
return(0);
}

/**************************************************************************
*
* Initialization of A16/D16 Card
*
***************************************************************************/
int devA16VmeConfig(int card, epicsUInt32 a16base, int nregs, int iVector, int iLevel, int iReg)
{
  epicsUInt32 probeVal;

  if(card < 0) {
      epicsPrintf("devA16VmeConfig: Invalid Card # %d \n",card);
      return(-1);
  }

  if(a16base >= 0x10000 || a16base & 0x1) {
    epicsPrintf("devA16VmeConfig: Invalid Card Address %d \n",a16base);
    return(-1);
  }

  if((a16base+(nregs*sizeof(epicsUInt16))) >= 0x10000 || nregs == 0) {
    epicsPrintf("devA16VmeConfig: Invalid # of registers\n");
    return(-1);
  }
  if (card >= num_cards) {
    struct ioCard *nc;
    nc = realloc(cards, sizeof(*cards)*(card+1)); 
    if(nc == NULL) {
      epicsPrintf("devA16VmeConfig: Can't allocate memory\n");
      return(-1);
    }
    cards = nc;
    while(num_cards <= card)
      cards[num_cards++].base = NULL;
  }
  if (cards[card].base != NULL) {
    epicsPrintf("devA16VmeConfig: Card %d allready configured\n", card);
    return(-1);
  }

  if(devRegisterAddress("apsA16Vme", atVMEA16, a16base, nregs*sizeof(epicsUInt16), (void*)&cards[card].base) != 0) {
       cards[card].base = NULL;
       epicsPrintf("devA16VmeConfig: A16 Address map failed for Card %d",card);
       return(-1);
  }

  if(devReadProbe( sizeof(short), (char *)cards[card].base, (char *)&probeVal) != 0 ) {
       cards[card].base = NULL;
       epicsPrintf("devA16VmeConfig: vxMemProbe failed for Card %d",card);
       return(-1);
  }

  cards[card].nReg = nregs;
 
  if(iVector) {
      scanIoInit(&cards[card].ioscanpvt);
      if(devConnectInterruptVME(iVector, devA16_isr, 
                    (int*)card) == 0)
      {
          cards[card].iReg = iReg;
          cards[card].base->reg[iReg] = (unsigned short)iVector;
          write_card(card, iReg, IVEC_MASK, (unsigned short)iVector);
          write_card(card, iReg, IVEC_ENABLE_MASK, IVEC_ENABLE_MASK);
          devEnableInterruptLevelVME(iLevel);
      }
      else {
          epicsPrintf("devA16VmeConfig: Interrupt connect failed for card %d\n",
                          card);
      }
      epicsAtExit(devA16RebootFunc, NULL);
  }       
  return(0);
}

/**************************************************************************
 *
 * BI record interrupt routine
 *
 **************************************************************************/
static long get_bi_int_info(int cmd, struct biRecord *pbi, IOSCANPVT *ppvt)
{

   struct vmeio           *pvmeio = (struct vmeio *)(&pbi->inp.value);

   if(cards[pvmeio->card].ioscanpvt != NULL) {
       *ppvt = cards[pvmeio->card].ioscanpvt;
       return(0);
   }
   else {
       return(-1);
   }
}


/**************************************************************************
 *
 * Interrupt service routine
 *
 **************************************************************************/
static void devA16_isr(void *arg)
{
   int card = (int)arg;
   scanIoRequest(cards[card].ioscanpvt);
   write_card(card, cards[card].iReg, IVEC_ENABLE_MASK, IVEC_ENABLE_MASK);
}


/******************************************************************************
 *
 * A function to disable interrupts in case we get a ^X style reboot.
 *
 ******************************************************************************/
static void devA16RebootFunc(void *arg)
{
  int   card = 0;

  while (card < num_cards)
  {
    if (cards[card].ioscanpvt != NULL) {
        write_card(card, cards[card].iReg, IVEC_ENABLE_MASK, 0);
    }
    card++;
  }
  return;
}

/**************************************************************************
 *
 * BO Initialization (Called one time for each BO MSLT card record)
 *
 **************************************************************************/
static long init_bo(struct boRecord *pbo)
{
    long                status = 0;
    int                 card, args, bit;
    epicsUInt32 	rawVal;
    a16VmeDpvt         *pPvt;

    /* bo.out must be an VME_IO */
    switch (pbo->out.type) {
    case (VME_IO) :
 
      card = pbo->out.value.vmeio.card;
      if(card >= num_cards) {
	pbo->pact = 1;		/* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pbo->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
	pbo->pact = 1;		/* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, pbo->name);
        return(-1);
      }

      if (pbo->out.value.vmeio.signal >= cards[card].nReg) {
        pbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pbo->name);
        return(-1);
      }

      args = sscanf(pbo->out.value.vmeio.parm, "%d", &bit);
 
      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pbo->name);
        return(-1);
      }

      pbo->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pbo->dpvt;

      pPvt->reg =  pbo->out.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = 1 << pPvt->lbit;
      pbo->mask = pPvt->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == 0)
         {
         pbo->rbv = pbo->rval = rawVal;
         }
      else 
         {
         status = 2;
         }
      break;
         
    default :
	pbo->pact = 1;		/* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal OUT field ->%s<- \n", pbo->name);
        status = -1;
    }
    return(status);
}

/**************************************************************************
 *
 * BI Initialization (Called one time for each BI record)
 *
 **************************************************************************/
static long init_bi(struct biRecord *pbi)
{
    long                status = 0;
    int                 card, args, bit;
    epicsUInt32         rawVal;
    a16VmeDpvt         *pPvt;
   

    /* bi.inp must be an VME_IO */
    switch (pbi->inp.type) {
    case (VME_IO) :

      card = pbi->inp.value.vmeio.card;
      if(card >= num_cards) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pbi->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",card, pbi->name);
        return(-1);
      }

      if (pbi->inp.value.vmeio.signal >= cards[card].nReg) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pbi->name);
        return(-1);
      }

      args = sscanf(pbi->inp.value.vmeio.parm, "%d", &bit);

      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pbi->name);
        return(-1);
      }
      pbi->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pbi->dpvt;

      pPvt->reg =  pbi->inp.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = 1 << pPvt->lbit;
      pbi->mask = pPvt->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == 0)
         {
         pbi->rval = rawVal;
         status = 0;
         }
      else
         {
         status = 2;
         }
      break;

    default :
        pbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal INP field ->%s<- \n", pbi->name);
        status = -1;
    }
    return(status);
}

/**************************************************************************
 *
 * MBBO Initialization (Called one time for each MBBO record)
 *
 **************************************************************************/
static long init_mbbo(struct mbboRecord *pmbbo)
{
    long                status = 0;
    int                 card, args, bit;
    epicsUInt32         rawVal;
    a16VmeDpvt         *pPvt;

    /* mbbo.out must be an VME_IO */
    switch (pmbbo->out.type) {
    case (VME_IO) :
      card = pmbbo->out.value.vmeio.card;
      if(card >= num_cards) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pmbbo->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, pmbbo->name);
        return(-1);
      }

      if (pmbbo->out.value.vmeio.signal >= cards[card].nReg) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pmbbo->name);
        return(-1);
      }

      args = sscanf(pmbbo->out.value.vmeio.parm, "%d", &bit);

      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pmbbo->name);
        return(-1);
      }

      pmbbo->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pmbbo->dpvt;

      pPvt->reg =  pmbbo->out.value.vmeio.signal;
      pPvt->lbit = bit;

      /* record support determined .mask from .nobt, need to adjust */
      pmbbo->shft = pPvt->lbit;
      pmbbo->mask <<= pPvt->lbit;
      pPvt->mask = pmbbo->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == 0)
         {
         pmbbo->rbv = pmbbo->rval = rawVal;
         status = 0;
         }
      else
         {
         status = 2;
         }
      break;

    default :
        pmbbo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal OUT field ->%s<- \n", pmbbo->name);
        status = -1;
    }
    return(status);
}

/**************************************************************************
 *
 * MBBI Initialization (Called one time for each MBBO record)
 *
 **************************************************************************/
static long init_mbbi(struct mbbiRecord *pmbbi)
{
    long                status = 0;
    int                 card, args, bit;
    epicsUInt32         rawVal;
    a16VmeDpvt         *pPvt;

    /* mbbi.inp must be an VME_IO */
    switch (pmbbi->inp.type) {
    case (VME_IO) :
      card = pmbbi->inp.value.vmeio.card;
      if(card >= num_cards) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pmbbi->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, pmbbi->name);
        return(-1);
      }

      if (pmbbi->inp.value.vmeio.signal >= cards[card].nReg) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pmbbi->name);
        return(-1);
      }

      args = sscanf(pmbbi->inp.value.vmeio.parm, "%d", &bit);

      if((args != 1) || (bit >= MAX_ACTIVE_BITS)) {
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit # in parm field: ->%s<-\n",
                     pmbbi->name);
        return(-1);
      }

      pmbbi->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pmbbi->dpvt;

      pPvt->reg =  pmbbi->inp.value.vmeio.signal;
      pPvt->lbit = bit;

      /* record support determined .mask from .nobt, need to adjust */
      pmbbi->shft = pPvt->lbit;
      pmbbi->mask <<= pPvt->lbit;
      pPvt->mask = pmbbi->mask;

      if (read_card(card, pPvt->reg, pPvt->mask, &rawVal) == 0)
         {
         pmbbi->rval = rawVal;
         status = 0;
         }
      else
         {
         status = 2;
         }
      break;

    default :
        pmbbi->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal INP field ->%s<- \n", pmbbi->name);
        status = -1;
    }
    return(status);
}

/**************************************************************************
 *
 * AI Initialization (Called one time for each AI record)
 *
 **************************************************************************/
static long init_ai(struct aiRecord *pai)
{
    long                status = 0;
    int                 card, args, bit, numBits;
    a16VmeDpvt         *pPvt;

    /* ai.inp must be an VME_IO */
    switch (pai->inp.type) {
    case (VME_IO) :
      card = pai->inp.value.vmeio.card;
      if(card >= num_cards) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pai->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, pai->name);
        return(-1);
      }

      if (pai->inp.value.vmeio.signal >= cards[card].nReg) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pai->name);
        return(-1);
      }

      args = sscanf(pai->inp.value.vmeio.parm, "%d,%d", &bit, &numBits);

      if((args != 2) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         (bit + numBits > MAX_ACTIVE_BITS)) {
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit #/Width in parm field: ->%s<-\n",
                     pai->name);
        return(-1);
      }

      pai->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pai->dpvt;

      pPvt->reg =  pai->inp.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      pai->eslo = (pai->eguf - pai->egul)/(pow(2,numBits)-1);
      
      status = 0;

      break;
    default :
        pai->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal INP field ->%s<- \n", pai->name);
        status = -1;
    }
    return(status);
}


/**************************************************************************
 *
 * AO Initialization (Called one time for each AO record)
 *
 **************************************************************************/
static long init_ao(struct aoRecord *pao)
{
    long                status = 0;
    epicsUInt32         rawVal;
    int                 card, args, bit, numBits;
    a16VmeDpvt         *pPvt;

    /* ao.out must be an VME_IO */
    switch (pao->out.type) {
    case (VME_IO) :
      card = pao->out.value.vmeio.card;
      if(card >= num_cards) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pao->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, pao->name);
        return(-1);
      }

      if (pao->out.value.vmeio.signal >= cards[card].nReg) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pao->name);
        return(-1);
      }

      args = sscanf(pao->out.value.vmeio.parm, "%d,%d", &bit, &numBits);

      if((args != 2) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         (bit + numBits > MAX_ACTIVE_BITS)) {
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit #/Width in parm field: ->%s<-\n",
                     pao->name);
        return(-1);
      }

      pao->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pao->dpvt;

      pPvt->reg =  pao->out.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      pao->eslo = (pao->eguf - pao->egul)/(pow(2,numBits)-1);

      /* Init rval to current setting */ 
      if(read_card(card,pPvt->reg,pPvt->mask,&rawVal) == 0) {
        pao->rbv = rawVal>>pPvt->lbit;
        pao->rval = pao->rbv;
      }

      status = 0;

      break;
    default :
        pao->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal OUT field ->%s<- \n", pao->name);
        status = -1;
    }
    return(status);
}

/**************************************************************************
 *
 * LI Initialization (Called one time for each LI record)
 *
 **************************************************************************/
static long init_li(struct longinRecord *pli)
{
    long                status = 0;
    int                 card, args, bit, numBits;
    a16VmeDpvt         *pPvt;

    /* li.inp must be an VME_IO */
    switch (pli->inp.type) {
    case (VME_IO) :
      card = pli->inp.value.vmeio.card;
      if(card >= num_cards) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, pli->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, pli->name);
        return(-1);
      }

      if (pli->inp.value.vmeio.signal >= cards[card].nReg) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     pli->name);
        return(-1);
      }

      args = sscanf(pli->inp.value.vmeio.parm, "%d,%d", &bit, &numBits);

      if((args != 2) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         (bit + numBits > MAX_ACTIVE_BITS)) {
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit #/Width in parm field: ->%s<-\n",
                     pli->name);
        return(-1);
      }

      pli->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)pli->dpvt;

      pPvt->reg =  pli->inp.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      status = 0;


      break;
    default :
        pli->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal INP field ->%s<- \n", pli->name);
        status = -1;
    }
    return(status);
}


/**************************************************************************
 *
 * Long Out Initialization (Called one time for each LO record)
 *
 **************************************************************************/
static long init_lo(struct longoutRecord *plo)
{
    long                status = 0;
    epicsUInt32         rawVal;
    int                 card, args, bit, numBits;
    a16VmeDpvt         *pPvt;

    /* lo.out must be an VME_IO */
    switch (plo->out.type) {
    case (VME_IO) :
      card = plo->out.value.vmeio.card;
      if(card >= num_cards) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d exceeds max: ->%s<- \n", card, plo->name);
        return(-1);
      }

      if(cards[card].base == NULL) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Card #%d not initialized: ->%s<-\n",
                     card, plo->name);
        return(-1);
      }

      if (plo->out.value.vmeio.signal >= cards[card].nReg) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Signal # exceeds registers: ->%s<-\n",
                     plo->name);
        return(-1);
      }

      args = sscanf(plo->out.value.vmeio.parm, "%d,%d", &bit, &numBits);

      if((args != 2) || (bit >= MAX_ACTIVE_BITS) || (numBits <= 0) ||
         (bit + numBits > MAX_ACTIVE_BITS)) {
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Invalid Bit #/Width in parm field: ->%s<-\n",
                     plo->name);
        return(-1);
      }

      plo->dpvt = (void *)calloc(1, sizeof(struct a16VmeDpvt));
      pPvt = (a16VmeDpvt *)plo->dpvt;

      pPvt->reg =  plo->out.value.vmeio.signal;
      pPvt->lbit = bit;
      pPvt->mask = ((1 << (numBits)) - 1) << pPvt->lbit;

      /* if .dol is NOT a constant, initialize .val field to readback value */
      if ((plo->dol.type == CONSTANT) && 
          (strlen(plo->dol.value.constantStr) == 0)) {
          if (read_card(card,pPvt->reg,pPvt->mask,&rawVal) == 0) {
              plo->val = rawVal>>pPvt->lbit;
              plo->udf = 0;
          }
      }

      status = 0;

      break;
    default :
        plo->pact = 1;          /* make sure we don't process this thing */
        epicsPrintf("devA16Vme: Illegal OUT field ->%s<- \n", plo->name);
        status = -1;
    }
    return(status);
}


/**************************************************************************
 *
 * Perform a write operation from a BO record
 *
 **************************************************************************/
static long write_bo(struct boRecord *pbo)
{

  epicsUInt32	 	rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pbo->dpvt;

  if (write_card(pbo->out.value.vmeio.card, pPvt->reg, pbo->mask, pbo->rval) 
        == 0)
  {
    if(read_card(pbo->out.value.vmeio.card, pPvt->reg, pbo->mask, &rawVal) 
        == 0)
    {
      pbo->rbv = rawVal;
      return(0);
    }
  }

  /* Set an alarm for the record */
  recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/**************************************************************************
 *
 * Perform a read operation from a BI record
 *
 **************************************************************************/
static long read_bi(struct biRecord *pbi)
{

  epicsUInt32           rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pbi->dpvt;

  if (read_card(pbi->inp.value.vmeio.card, pPvt->reg, pbi->mask,&rawVal) == 0)
  {
     pbi->rval = rawVal;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pbi, READ_ALARM, INVALID_ALARM);
  return(2);
}


/**************************************************************************
 *
 * Perform a read operation from a MBBI record
 *
 **************************************************************************/
static long read_mbbi(struct mbbiRecord *pmbbi)
{

  epicsUInt32           rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pmbbi->dpvt;

  if (read_card(pmbbi->inp.value.vmeio.card,pPvt->reg,pmbbi->mask,&rawVal) 
        == 0)
  {
     pmbbi->rval = rawVal;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pmbbi, READ_ALARM, INVALID_ALARM);
  return(2);
}


/**************************************************************************
 *
 * Perform a write operation from a MBBO record
 *
 **************************************************************************/
static long write_mbbo(struct mbboRecord *pmbbo)
{

  epicsUInt32           rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pmbbo->dpvt;

  if (write_card(pmbbo->out.value.vmeio.card,pPvt->reg,
                      pmbbo->mask,pmbbo->rval) == 0)
  {
    if(read_card(pmbbo->out.value.vmeio.card,pPvt->reg,pmbbo->mask,&rawVal) 
       == 0)
    {
      pmbbo->rbv = rawVal;
      return(0);
    }
  }

  /* Set an alarm for the record */
  recGblSetSevr(pmbbo, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/**************************************************************************
 *
 * Perform a read operation from a AI record
 *
 **************************************************************************/
static long read_ai(struct aiRecord *pai)
{

  epicsUInt32           rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pai->dpvt;

  if (read_card(pai->inp.value.vmeio.card,pPvt->reg,pPvt->mask,&rawVal) == 0)
  {
     pai->rval = rawVal>>pPvt->lbit;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
  return(2);

}

/**************************************************************************
 *
 * Perform a write operation from a AO record
 *
 **************************************************************************/
static long write_ao(struct aoRecord *pao)
{

  epicsUInt32           rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pao->dpvt;

  if (write_card(pao->out.value.vmeio.card,pPvt->reg,
                 pPvt->mask,pao->rval<<pPvt->lbit) == 0)
  {
    if(read_card(pao->out.value.vmeio.card,pPvt->reg,pPvt->mask,&rawVal)
       == 0)
    {
      pao->rbv = rawVal>>pPvt->lbit;
      return(0);
    }
  }

  /* Set an alarm for the record */
  recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/**************************************************************************
 *
 * Perform a read operation from a LI record
 *
 **************************************************************************/
static long read_li(struct longinRecord *pli)
{

  epicsUInt32           rawVal;
  a16VmeDpvt           *pPvt = (a16VmeDpvt *)pli->dpvt;

  if (read_card(pli->inp.value.vmeio.card,pPvt->reg,pPvt->mask,&rawVal) == 0)
  {
     pli->val = rawVal>>pPvt->lbit;
     pli->udf = 0;
     return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(pli, READ_ALARM, INVALID_ALARM);
  return(2);

}

/**************************************************************************
 *
 * Perform a write operation from a LO record
 *
 **************************************************************************/
static long write_lo(struct longoutRecord *plo)
{

  a16VmeDpvt           *pPvt = (a16VmeDpvt *)plo->dpvt;

  if (write_card(plo->out.value.vmeio.card,pPvt->reg,
                 pPvt->mask,plo->val<<pPvt->lbit) == 0)
  {
      return(0);
  }

  /* Set an alarm for the record */
  recGblSetSevr(plo, WRITE_ALARM, INVALID_ALARM);
  return(2);
}

/**************************************************************************
 *
 * Raw read a bitfield from the card
 *
 **************************************************************************/
static long read_card(short card, epicsUInt16 reg, epicsUInt32 mask, epicsUInt32 *value)
{
  if (checkCard(card) == -1)
    return(-1);

  *value = cards[card].base->reg[reg] & mask;

  if (devA16VmeDebug >= 20)
    printf("devA16Vme: read 0x%4.4X from card %d\n", *value, card);

  return(0);
}

/**************************************************************************
 *
 * Write a bitfield to the card retaining the states of the other bits
 *
 **************************************************************************/
static long write_card(short card, epicsUInt16 reg, epicsUInt32 mask, epicsUInt32 value)
{
  int key;

  if (checkCard(card) == -1)
    return(-1);

  key = epicsInterruptLock();
  cards[card].base->reg[reg] = ((cards[card].base->reg[reg] & ~mask) | 
                              (value & mask));
  epicsInterruptUnlock(key);

  if (devA16VmeDebug >= 15)
    printf("devA16Vme: wrote 0x%4.4X to card %d\n",
            cards[card].base->reg[reg], card);

  return(0);
}

/**************************************************************************
 *
 * Make sure card number is valid
 *
 **************************************************************************/
static long checkCard(short card)
{
  if ((card >= num_cards) || (cards[card].base == NULL))
    return(-1);
  else
    return(0);
}

/* devA16VmeConfig ioc shell callable wrapper */
static const iocshArg A16Vme_configureArg0 = {"card", iocshArgInt};
static const iocshArg A16Vme_configureArg1 = {"a16base", iocshArgInt};
static const iocshArg A16Vme_configureArg2 = {"nreg", iocshArgInt};
static const iocshArg A16Vme_configureArg3 = {"iVector", iocshArgInt};
static const iocshArg A16Vme_configureArg4 = {"iLevel", iocshArgInt};
static const iocshArg A16Vme_configureArg5 = {"iReg", iocshArgInt};

static const iocshArg * const A16Vme_configureArgs[] = {&A16Vme_configureArg0,&A16Vme_configureArg1,&A16Vme_configureArg2,&A16Vme_configureArg3,&A16Vme_configureArg4,&A16Vme_configureArg5};
static const iocshFuncDef A16Vme_configureFuncDef = {"devA16VmeConfig",6,A16Vme_configureArgs};
static void devA16VmeConfigCallFunc(const iocshArgBuf *args)
{
   devA16VmeConfig(args[0].ival,args[1].ival,args[2].ival,args[3].ival,args[4].ival,args[5].ival);
}

static void epicsShareAPI drvA16VmeRegistrar(void)
{
   iocshRegister(&A16Vme_configureFuncDef, devA16VmeConfigCallFunc);
}
epicsExportRegistrar(drvA16VmeRegistrar);
