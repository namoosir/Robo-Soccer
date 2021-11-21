/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 1.1 - Updated Aug 2019 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!

//global var
double ball_x, ball_y, ballm_x = 0, ballm_y = 0;
double opp_x, opp_y;
double self_x, self_y, self_angle;
int go_buffer = 0;
int kick_buffer = 230;
int kick_ready = 0;
int kicking = 0;
double onetx = 0;
double onety = 0;
double snetx = 0;
double snety = 0;

double dottie(double vx, double vy, double ux, double uy)
{
 // Returns the dot product of the two vectors [vx,vy] and [ux,uy]
 return (vx*ux)+(vy*uy);
}

double crossie_sign(double vx, double vy, double ux, double uy)
{
 // Returns the sign of the Z component of the cross product of 
 //   vectors [vx vy 0] and [ux uy 0]
 // MIND THE ORDER! v rotating onto u.     
 // i   j   k 
 // vx  vy  0
 // ux  uy  0
    
 if ((vx*uy)-(ux*vy)<0) return -1;
 else return 1;
}

/**************************************************************
 * Display List Management - EXPERIMENTAL (you don't need this
 * to solve your RoboSoccer project). The Display List is an
 * experimental feature to allow you to draw things on the
 * display window. However, it's not (yet) fully tested and 
 * you can easily get into a segfault.
 * ** USE AT YOUR OWN RISK **
 * ***********************************************************/
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type=0;
  newNode->x1=x;
  newNode->y1=y;
  newNode->x2=-1;
  newNode->y2=-1;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  
  newNode->next=head;
  return(newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x2;
  newNode->y2=y2;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);  
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;
  
  l=sqrt((dx*dx)+(dy*dy));
  dx=dx/l;
  dy=dy/l;
  
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addVector(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x1+(length*dx);
  newNode->y2=y1+(length*dy);
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x-length;
  newNode->y1=y;
  newNode->x2=x+length;
  newNode->y2=y;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  head=newNode;

  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x;
  newNode->y1=y-length;
  newNode->x2=x;
  newNode->y2=y+length;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while(head)
  {
      q=head->next;
      free(head);
      head=q;
  }
  return(NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Red
 //                   1 -> Yellow or Green (see below)
 //                   2 -> Blue
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.65;                                              // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // The reference colours here are in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : Own bot,  1 : Opponent's bot, 2 : Ball
  if (col==0) {vr_x=cos(0); vr_y=sin(0);}                   // RED                                
  else if (col==2) {vr_x=cos(2.0*PI*(60.0/360.0));
                    vr_y=sin(2.0*PI*(60.0/360.0));}         // YELLOW
  else if (col==1) {vr_x=cos(2.0*PI*(240.0/360.0));
                   vr_y=sin(2.0*PI*(240.0/360.0));}         // BLUE

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {

  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Motion direction vector. Not valid
 //   while rotating - possibly valid while turning
 // - Heading direction - vector obtained from the blob shape, it is
 //   correct up to a factor of (-1) (i.e. it may point backward w.r.t.
 //   the direction your bot is facing). This vector remains valid
 //   under rotation.
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;

 // Reset ID flags
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;

 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);       // With 2, Ball is set to be yellow
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;
  ai->st.bdx=p->dx;
  ai->st.bdy=p->dy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute motion direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  else
  {
    ai->st.bmx=0;
    ai->st.bmy=0;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;

  ball_x = ai->st.ball->cx;
  ball_y = ai->st.ball->cy;
  ballm_x = ai->st.bmx;
  ballm_y = ai->st.bmy;

 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);     // If 0, our bot is BLUE, if 1 our bot is RED
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob

  // Adjust Y position if we have calibration data
  if (fabs(p->adj_Y[0][0])>.1)
  {
   dmax=384.0-p->adj_Y[0][0];
   dmin=767.0-p->adj_Y[1][0];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][0]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;
  ai->st.sdx=p->dx;
  ai->st.sdy=p->dy;

  if(ai->st.state%100 !=0){
    double temp_angle = get_angle_from_vector(ai->st.sdx, ai->st.sdy);
    double flipped_angle = fmod((temp_angle + 180), 360);
    double self_error = temp_angle - self_angle;
    double flipped_error = flipped_angle - self_angle;

    if(self_error < -180){
      self_error += 360;
    } else if (self_error > 180) {
      self_error += -360;
    }
    if(flipped_error < -180){
      flipped_error += 360;
    } else if (flipped_error > 180) {
      flipped_error += -360;
    }
    
    if (fabs(self_error) < fabs(flipped_error)) {
      self_angle = temp_angle;
    } else {
      self_angle = flipped_angle;
    }
  }
  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));

  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }
  else
  {
   ai->st.smx=0;
   ai->st.smy=0;
  }
  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,0);
 else p=id_coloured_blob2(ai,blobs,1);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	

  if (fabs(p->adj_Y[0][1])>.1)
  {
   dmax=384.0-p->adj_Y[0][1];
   dmin=767.0-p->adj_Y[1][1];
   pink=(dmax-dmin)/(768.0-384.0);
   adj=dmin+((p->adj_Y[1][1]-p->cy)*pink);
   p->cy=p->cy+adj;
   if (p->cy>767) p->cy=767;
   if (p->cy<1) p->cy=1;
  }

  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;
  ai->st.odx=p->dx;
  ai->st.ody=p->dy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  else
  {
   ai->st.omx=0;
   ai->st.omy=0;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 double frame_inc=1.0/10.0;

 BT_drive(MOTOR_B, MOTOR_C, 30);			// Start forward motion to establish heading
					// Will move for a few frames.

 track_agents(ai,blobs);		// Call the tracking function to find each agent

 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI 
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.sdx=0;
 ai->st.sdy=0;
 ai->st.odx=0;
 ai->st.ody=0;
 ai->st.bdx=0;
 ai->st.bdy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 ai->DPhead=NULL;
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 // The code here just makes sure the image processing loop is constantly
 // tracking the bots while they're placed in the locations required
 // to do the calibration (i.e. you DON'T need to add anything more
 // in this function).
 track_agents(ai,blobs);
}


/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static double ux,uy,len,mmx,mmy,px,py,lx,ly,mi;
  double angDif, lPow,rPow;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;
  struct displayList *q;
  
  // change to the ports representing the left and right motors in YOUR robot
  char lport=MOTOR_B;
  char rport=MOTOR_C;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
    
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	// The id_bot() routine will change the AI state to initial state + 1
  {				// if robot identification is successful.
   if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;
   BT_all_stop(0);
   
   fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
   
   if (ai->st.self!=NULL)
   {
       // This checks that the motion vector and the blob direction vector
       // are pointing in the same direction. If they are not (the dot product
       // is less than 0) it inverts the blob direction vector so it points
       // in the same direction as the motion vector.
       if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
       {
       

           ai->st.self->dx*=-1.0;
           ai->st.self->dy*=-1.0;
           ai->st.sdx*=-1;
           ai->st.sdy*=-1;
       }
       old_dx=ai->st.sdx;
       old_dy=ai->st.sdy;
       if(ai->st.smx != 0 && ai->st.smx != 0){
        self_angle = get_angle_from_vector(old_dx, old_dy);
       }

       if (ai->st.side==1) {
         onetx = 0;
         onety = 360;
         snetx = 1050;
         snety = 360;
       } else {
         onetx = 1050;
         onety = 360;
         snetx = 0;
         snety = 360;
       }
   }
  
   if (ai->st.opp!=NULL)
   {
       // Checks motion vector and blob direction for opponent. See above.
       if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
       {
           ai->st.opp->dx*=-1;
           ai->st.opp->dy*=-1;
           ai->st.odx*=-1;
           ai->st.ody*=-1;
       }       
   }

   // Notice that the ball's blob direction is not useful! only its
   // position and motion matter.
  }
     
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
//  fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
  track_agents(ai,blobs);		// Currently, does nothing but endlessly track
  printf("state:%d\n", ai->st.state);
  printf("x: %f y: %f angle: %f\n", ball_x, ball_y, self_angle);
  // printf("nx: %f ny: %f \n", onetx, onety);

  if (ai->st.state >= 1 && ai->st.state <= 99) {
    //TODO
  } else if (ai->st.state >= 101 && ai->st.state <= 199) {
    // if (!ai->st.selfID) {
    //   // state = 100;
    //   ai->st.state = 100;
    //   return;
    // } else
     if (ai->st.state == 101) {
      if(kick_ready>0){
        ai->st.state == 107;
      }
      go_behind_ball(ai, 102);

    } else if (ai->st.state == 102) {
      if(kick_ready>0){
        ai->st.state == 107;
      }
      go_to_ball_by_buffer(ai, 103);

    } else if (ai->st.state == 103) {
      if(kick_ready>0){
        ai->st.state == 107;
      }
      face_to_ball(ai, 104, 9);
    } else if (ai->st.state == 104) {
      go_to_ball_by_buffer(ai, 105);
      return; 
    } else if (ai->st.state == 105) {
      heavy_kick(ai, 106);
      // recover_kick(ai, 106);
      return;
    } else if (ai->st.state == 106) {
      recover_kick(ai, 107);
      return;
    } else if (ai->st.state == 107) {
      return;
    }
  } else if (ai->st.state >= 201){
    // if (!ai->st.ballID || !ai->st.selfID) {
    //   // printf("can't find agents\n");
    //   // state = 200;
    //   ai->st.state = 200;
    //   return;
    // } else 
    if (ai->st.state == 201) {
      // printf("going towards ball\n");
      if(kick_ready>0){
        ai->st.state == 204;
      }
      go_towards_ball(ai, 202);
      return;
    } else if (ai->st.state == 202) {
      if(kick_ready>0){
        ai->st.state == 204;
      }
      go_to_ball_by_buffer(ai, 203);
      return;
    } else if(ai->st.state == 203){
      light_kick(ai, 204);
      return;
    } else if(ai->st.state == 204){
      recover_kick(ai, 201);
      return;
    }
  } else {
    // printf("state: %d\n", ai->st.state);
    return;
  }
 }

}

void go_behind_ball(struct RoboAI *ai, int back_to_state) {
  double distance_from_ball = 130;

  //ball to opponent net vector
  double nbx = ball_x - onetx;
  double nby = ball_y - onety;

  double unbx, unby;
  unit_vector(&unbx, &unby, nbx, nby);
  double targetx = ball_x + unbx*distance_from_ball;
  double targety = ball_y + unby*distance_from_ball;
  
  if(compare_by_side(ai, targetx, ai->st.old_scx)){
    go_to_location(ai, targetx, targety, back_to_state, 6);

  }
}

void face_to_ball(struct RoboAI *ai, int back_to_state, int go_buff){
  double error = get_angle_from_vector(onetx-ai->st.old_scx, onety-ai->st.old_scy) - self_angle;

  if (error < -180) {
    error += 360;
  } else if (error > 180) {
    error += -360;
  }
  if (fabs(error)>3) {
    BT_all_stop(1);

    int power = 10;
    if(fabs(error)>60){
      power = 30;
    }
    if(error>0){
      BT_motor_port_start(MOTOR_B,power);
      BT_motor_port_start(MOTOR_C,-power);
    }else{
      BT_motor_port_start(MOTOR_B,-power);
      BT_motor_port_start(MOTOR_C,power);
    }
    return;
  }else{
    BT_all_stop(0);
    go_buffer = go_buff;
    ai->st.state = back_to_state;
  }
}

int compare_by_side(struct RoboAI *ai, double var1, double var2){
  if(ai->st.side == 0){
    return var1 > var2;
  }
  return var1 > var2;
}

//go to the ball after facing the ball and starting from the self net
void go_to_ball(struct RoboAI *ai, struct blob *blobs, void* state) {
  // //opponent net NEED TO GET
  // double onetx = 0;
  // double onety = 0;

  // //ball to opponent net vector
  // double bnx = onetx - ai->st.ball->cx;
  // double bny = onety - ai->st.ball->cy;

  //self to ball vector
  double sbx = ball_x - ai->st.self->cx;
  double sby = ball_y - ai->st.self->cy;

  //self to ball vector error in angle
  double sb_error = self_angle - get_angle_from_vector(sbx, sby);
  //dist from self to ball
  double dist_to_ball = dist(ai->st.self->cx, ai->st.self->cy, ai->st.ball->cx, ai->st.ball->cy);
  int distance_threshold = 100;

  if (sb_error < -180) {
    sb_error += 360;
  } else if (sb_error > 180) {
    sb_error += -360;
  }

  if (dist_to_ball < distance_threshold && sb_error < 10 && sb_error > -10) {
    ai->st.state = 103;
    return;
  } else if (sb_error > 10 || sb_error < -10) {
    BT_all_stop(1);

    double c = 100;
    BT_motor_port_start(MOTOR_B,(c*sb_error)/sb_error);
    BT_motor_port_start(MOTOR_C,(c*-sb_error)/sb_error);
    return;
  } else {
    BT_all_stop(1);
    BT_drive(MOTOR_B, MOTOR_C, 100);
    return;
  }
}

//go to self net and then face the ball
void go_to_self_net(struct RoboAI *ai, struct blob *blobs, void* state) {
  //self net NEED TO GET
  double snetx = 0;
  double snety = 0;

  //self to ball vector
  double sbx = ball_x - ai->st.old_scx;
  double sby = ball_y - ai->st.old_scy;

  //self to self net vector
  double ssnx = snetx - ai->st.old_scx;
  double ssny = snety - ai->st.old_scy;

  //self to self net vector error in angle
  double ssn_error = self_angle - get_angle_from_vector(ssnx, ssny);
  //dist from self to self net
  double dist_to_self_net = dist(ai->st.old_scx, ai->st.old_scy, snetx, snety);
  double sb_error = get_angle_from_vector(sbx, sby) - get_angle_from_vector(ai->st.self->cx, ai->st.self->cy);

  int distance_threshold = 100;
  
  if (ssn_error < -180) {
    ssn_error += 360;
  } else if (ssn_error > 180) {
    ssn_error += -360;
  }

  if (sb_error < -180) {
    sb_error += 360;
  } else if (sb_error > 180) {
    sb_error += -360;
  }

  if (dist_to_self_net < distance_threshold && sb_error < 10 && sb_error > -10) {
    ai->st.state = 102;
    return;
  } else if (dist_to_self_net < distance_threshold) {
    BT_all_stop(1);

    //turn and face the ball
    double c = 4;
    BT_motor_port_start(MOTOR_B,(c*ssn_error)/100);
    BT_motor_port_start(MOTOR_C,(c*-ssn_error)/100);
    return;
  } else if (ssn_error > 10 || ssn_error < -10) {
    BT_all_stop(1);

    double c = 100;
    BT_motor_port_start(MOTOR_B,(c*ssn_error)/ssn_error);
    BT_motor_port_start(MOTOR_C,(c*-ssn_error)/ssn_error);
    return;
  } else {
    BT_all_stop(1);
    BT_drive(MOTOR_B, MOTOR_C, 100);
    return;
  }

}

void heavy_kick(struct RoboAI *ai, int back_to_state) {
  if(kick_buffer>0){
    BT_motor_port_start(MOTOR_A, -100);
    kick_buffer -= 1;
  }else{
    BT_all_stop(0);
    kick_ready = 15;
    ai->st.state = back_to_state;
  }
}

//might need to adjust using blob->adj_Y
void go_towards_ball(struct RoboAI *ai, int back_to_state) {
  double ball_pred_x = ball_x + ballm_x;
  double ball_pred_y = ball_y + ballm_y;

  go_to_location(ai, ball_pred_x, ball_pred_y, back_to_state, 4);
}

void go_to_location(struct RoboAI *ai, double x, double y, int back_to_state, int go_buff) {
  double error = get_angle_from_vector(x-ai->st.old_scx, y-ai->st.old_scy) - self_angle;
  int distance_threshold = 130;

  if (error < -180) {
    error += 360;
  } else if (error > 180) {
    error += -360;
  }
  double d = dist(x, y, ai->st.old_scx, ai->st.old_scy);
  
  printf("targetx: %f, targety: %f distance : %f\n", x, y, d);
  //close to ball
  if (d < distance_threshold && error < 10 && error > -10) {
    BT_all_stop(1);
    // printf("close to ball\n");
    go_buffer = go_buff;
    kick_buffer = 10;
    BT_all_stop(0);
    ai->st.state = back_to_state;
    return;
  //not facing ball
  } else if (error > 10 || error < -10) {
    BT_all_stop(1);

    int power = 10;
    if(fabs(error)>60){
      power = 30;
    }
    if(error>0){
      BT_motor_port_start(MOTOR_B,power);
      BT_motor_port_start(MOTOR_C,-power);
    }else{
      BT_motor_port_start(MOTOR_B,-power);
      BT_motor_port_start(MOTOR_C,power);
    }
    
    return;
  //facing ball but not close to it
  } else {
    // printf("facing ball but not close to it\n");
    BT_all_stop(1);
    BT_drive(MOTOR_B, MOTOR_C, 50);
    return;
  }
}

void light_kick(struct RoboAI *ai, int back_to_state) {
  if(kick_buffer>0){
    BT_motor_port_start(MOTOR_A, -50);
    kick_buffer -= 1;
  }else{
    BT_all_stop(0);
    kick_ready = 10;
    ai->st.state = back_to_state;
  }
}

void recover_kick(struct RoboAI *ai, int back_to_state) {
  BT_all_stop(0);
  if(kick_ready>0){
    BT_motor_port_start(MOTOR_A, 50);
    kick_ready = kick_ready-1;
  }else{
    BT_all_stop(0);
    ai->st.state = back_to_state;
  }
}

void go_to_ball_by_buffer(struct RoboAI *ai, int back_to_state) {
  BT_all_stop(0);
  if(go_buffer>0){
    BT_drive(MOTOR_B, MOTOR_C, 40);
    go_buffer -= 1;
  }else{
    BT_all_stop(0);
    ai->st.state = back_to_state;
  }
}
/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/


// kick the ball with power power
void kick_ball(int power) {
  int time = 10000/power;

  for (int i = 0; i < time; i++) {
    BT_motor_port_start(MOTOR_A, power);
  }

  BT_all_stop(1);
  kick_ready=0;
}

double dist(double self_x, double self_y, double target_x, double target_y) {
  return sqrt(powf(self_x-target_x, 2) + powf(self_y-target_y, 2));
}

double get_angle_from_vector(double x, double y) {
  double angle=0;
  if(x==0){
    if(y>0){
      angle = 180;
    }else{
      angle = 0;
    }
  }else if (x>0) {
    if(y==0){
      angle = 90;
    }else{
      angle = 90+atan(y/x) * 90 / (PI/2);
    }
  }else if(x<0){
    if(y==0){
      angle = 270;
    }else{
      angle = 270+atan(y/x) * 90 / (PI/2);
    }
  }
  return angle;
}

void unit_vector(double* out_x, double* out_y, double x, double y) {
  double norm = dist(x, y, 0, 0);
  *out_x = x/norm;
  *out_y = y/norm;
}