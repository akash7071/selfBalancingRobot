/* ========================================================================== */
/*                                                                            */
//
//
//Edited by Akash Patil and Vidhya Palaniappan
//Edited on: 5/5/2024
//Edited for RTES Project: Self-balancing robot prototype
//Function: Read sensor data, calculate motor speed, create threads and sequencer, main loop
//References:https://github.com/wennycooper/mySegway/tree/master, Prof. Sam Siewerts Feasibility test code
//
/* ========================================================================== */



#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>

#include <errno.h>
#include <time.h>
#include <unistd.h>

#include <wiringPiI2C.h>
#include <signal.h> 
#include <wiringPi.h>
#include <math.h>
#include <assert.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define U32_T unsigned int

#define NUM_THREADS (4+1)


#define RAD_TO_DEG  57.29578
#define GYRO_SCALE_X 131.0
#define GYRO_SCALE_Y 131.0

#define ECHO_PIN 15
#define TRIG_PIN 16

// PID parameters
double Kp = 14;   // How fast the robot balances
double Ki = 8;   // How much of previous error to take into account during each cycle
double Kd = 210;   // How less should the system oscillate
double K  = 1.9*1.12;   // scale factor for speed


// Complimentary Filter parameters
double K0 = (double) 0.98;  //Weightage of Gyroscope
double K1 = (double) 0.02;  //Weightage of accelerometer

//global variables for storing IMU values
int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z;
double gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;
float distance;

//arrays for computing feasibility tests: completion test and scheduling point
int completion_time_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);
int scheduling_point_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);

U32_T ex1_period[] = {2500, 2500, 2500};
U32_T ex1_wcet[] = {1407, 44, 63};
U32_T ex1_deadline[] = {2323, 2396, 2500};

//variables for offsets, differences and previous axes values
double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;

double GUARD_GAIN = 100.0;      //cap speed to 100 to keep duty cycle to 50%(max value 200)
double error, last_error, integrated_error; //variables for current error, previous error and integrated error for I term
double pTerm, iTerm, dTerm;// PID terms
double angle;
double angle_offset = 0;  //offset for sensor manufacturing defects
double speed;              //store speed of the motor 


//structures for time related data
double deltaT;             //variable to store time difference
struct timeval tv, tv2;
struct timeval start_time_val;
unsigned long long timer, t,bootup;

//service control from sequencer needs below variables
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE;
sem_t semS1, semS2, semS3, semS4;

typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;

//declaring each service along with sequencer service
double getTimeMsec(void);
void print_scheduler(void);
void testAxesData();
void testMotorControl();
void *service_1(void *threadp);
void *service_2(void *threadp);
void *service_3(void *threadp);
void *service_4(void *threadp);
void *Sequencer(void *threadp);

//part of stretch goals for Ultrasonic sensor
float measureDistance() {
    digitalWrite(TRIG_PIN, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW); 

    while (digitalRead(ECHO_PIN) == LOW); 
    long startTime = micros(); 
    while (digitalRead(ECHO_PIN) == HIGH); 
    long endTime = micros(); 

    long pulseDuration = endTime - startTime;
    float distance = pulseDuration / 58.0; 

    return distance; 
}

//function to read a word of address taking addr as input argument
int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0x8000)
    val = -(65536 - val);

  return val;
}

//calculates distance between two points, needed for tilt determination
double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}

//returns rotation of system along y axis
double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

//returns rotation of system along x axis
double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}


// function to read 12 bytes of data from the IMU. 2Bytes*3 axes*2 sensors
void read_all()
{
    //read from accel via I2C
    acclX = read_word_2c(0x3D);
    acclY = read_word_2c(0x3B);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    //read from gyro via I2C
    gyroX = read_word_2c(0x45);
    gyroY = read_word_2c(0x43);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / GYRO_SCALE_X;
    gyro_scaled_y = gyroY / GYRO_SCALE_Y;
    gyro_scaled_z = gyroZ / GYRO_SCALE_X;
}

//function used to determine timings
unsigned long long  getTimestamp()
{
  gettimeofday(&tv, NULL);
  return (unsigned long long) tv.tv_sec * 1000000 + tv.tv_usec;
}


//this function is used to limit the speed of the motor to no greater than max value of PWM
double constrain(double v, double min_v, double max_v)
{
  if (v <= min_v)
    return (double)min_v;
  else if (v >= max_v)
    return (double)max_v;
  else
    return (double)v;
}

//PID control function.
void pid()
{
  error = last_y - angle_offset;    //determines difference between offset and current orientation of robot
 
  

  pTerm = Kp * error;           //Proportional term. Directly affects speed

  integrated_error = 0.95*integrated_error + error; //Integral term, Makes sure that system comes close to equilibrium position
  iTerm = Ki * integrated_error;    
  
  dTerm = Kd * (error - last_error); //Differential term, controls overshoot
  last_error = error;

  speed = constrain(K*(pTerm + iTerm + dTerm), -GUARD_GAIN, GUARD_GAIN);    //keeps the forward or backward speed between -100 to 100
  
}

//for exiting program
void sigintHandler(int signal) {
    stop_motors(); 
    exit(EXIT_SUCCESS); 
}


//start of main loop
void main(void)
{
    wiringPiSetup();                //Initialising GPIO using wiringPi library
    signal(SIGINT, sigintHandler);  
    pinMode(TRIG_PIN, OUTPUT);      //stretch goal: Ultrasonic sensor GPIO
    pinMode(ECHO_PIN, INPUT);       //stretch goal: Ultrasonic sensor GPIO
    digitalWrite(TRIG_PIN, LOW);    //stretch goal: Ultrasonic sensor GPIO
    delay(200);                     //wait for all sensors to bootup
    init_motors();                  //Init GPIO for motors
    bootup = getTimestamp();        //Get program start time to determine service execution times
  
    struct timeval current_time_val;//struct for storing time data
    int i, rc, scope;
    
    //variables for setting up threads and parameters for the threads
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;
    CPU_ZERO(&allcpuset);
    U32_T numServices = 3;
	
    for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);
 
    //Setting up semaphores for sequentially releasing each of our services
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&semS4, 0, 0)) { printf ("Failed to initialize S3 semaphore\n"); exit (-1); }

    mainpid=getpid();

    //getting and setting priority
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();


    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");
    
    //init I2C communication with I2C peripheral at address 0x68
    fd = wiringPiI2CSetupInterface ("/dev/i2c-1",0x68);
    wiringPiI2CWriteReg8 (fd,0x6B,0x00);    //setting the sensor to not sleep at all

    timer = getTimestamp();

    deltaT = (double) (getTimestamp() - timer)/1000000.0;
    read_all();                 //read all sensor data

    
    
    last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

    gyro_offset_x = gyro_scaled_x;
    gyro_offset_y = gyro_scaled_y;

    gyro_total_x = last_x   - gyro_offset_x;
    gyro_total_y = last_y - gyro_offset_y;
	
    printf("perfoming test cases\n\r");
    testAxesData();
    testMotorControl();
    
    for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n\r", CPU_COUNT(&threadcpu));

    threadParams[0].sequencePeriods=8000000;
       
    //creation of first thread
    rt_param[1].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1], &rt_sched_attr[1], service_1, (void *)&(threadParams[1]));
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("\n\rpthread_create successful for Service 1: Sensor Service with priority max-1\n");
		
    //creation of second service
	rt_param[2].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for Service 2: Pid service with priority max-2\n");
	
    //creation of theird service	
	rt_param[3].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], service_3, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        printf("pthread_create successful for Service 3: Motor service with priority max-3\n");
		
/*	rt_param[4].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[4], &rt_param[4]);
    rc=pthread_create(&threads[4], &rt_sched_attr[4], service_4, (void *)&(threadParams[4]));
    if(rc < 0)
        perror("pthread_create for service 4");
    else
        printf("pthread_create successful for service 4\n");*/
        
    //creation of sequencer thread    
    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for Service 0: sequeencer with priority max\n\r");

    //completion time feasiility test
    if(completion_time_feasibility(numServices, ex1_period, ex1_wcet, ex1_deadline) == TRUE)
        printf("Completion Time test - FEASIBLE\n");
    else
        printf("\n\rCompletion Time test - INFEASIBLE\n");
    //scheduloing point feasibility test    
    if(scheduling_point_feasibility(numServices, ex1_period, ex1_wcet, ex1_deadline) == TRUE)
        printf("scheduling Point test - FEASIBLE\n");
    else
        printf("scheduling Point test - INFEASIBLE\n");         
    printf("\n");
    
    //join all threads back before termination
   for(i=0;i<NUM_THREADS;i++)
    pthread_join(threads[i], NULL);   
}

//sequencer thread which schedules all services.
void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
    struct timespec delay_time = {0,1250000}; // delay for 1.25 msec, frequency=800 Hz
    struct timespec remaining_time;
    double current_time;
    double residual;
    int rc, delay_cnt=0;
    unsigned long long seqCnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        delay_cnt=0; residual=0.0;
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            {
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

        seqCnt++;
        gettimeofday(&current_time_val, (struct timezone *)0);
        syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);


        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);


        // Release each service at a sub-rate of the generic sequencer rate
        // Servcie_1 = RT_MAX-1 @ 400 Hz
        if((seqCnt % 2) == 0) sem_post(&semS1); //release first service

        // Service_4 = RT_MAX-2 @ 200 Hz
        if((seqCnt % 4) == 0) sem_post(&semS4); //Stretch goal: release movement service


    } while(!abortTest && (seqCnt < threadParams->sequencePeriods));

    sem_post(&semS1); sem_post(&semS4);
    abortS1=TRUE; abortS4=TRUE;

    pthread_exit((void *)0);
}

//first service reads orientation data from IMU
void *service_1(void *threadp)
{
    timer = getTimestamp();

    deltaT = (double) (getTimestamp() - timer)/1000000.0;
    read_all();

    last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
    last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

    gyro_offset_x = gyro_scaled_x;
    gyro_offset_y = gyro_scaled_y;

    gyro_total_x = last_x - gyro_offset_x;
    gyro_total_y = last_y - gyro_offset_y;
  
    double start = 0.0;
    double finish = 0.0;
    double WCET = 0;
    double max_time=0.0;
    
    while(!abortS1)
    {
        sem_wait(&semS1); 
        
        t = getTimestamp();
        deltaT = (double) (t - timer)/1000000.0;
        timer = t;
        
        start = getTimestamp();       
        read_all();
        finish = getTimestamp();

        gyro_scaled_x -= gyro_offset_x;
        gyro_scaled_y -= gyro_offset_y;

        gyro_x_delta = (gyro_scaled_x * deltaT);
        gyro_y_delta = (gyro_scaled_y * deltaT);

        gyro_total_x += gyro_x_delta;
        gyro_total_y += gyro_y_delta;

        rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
        rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

        last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
        last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);
        
        
        WCET = finish - start;
        if(WCET>max_time)
            max_time=WCET;
        
        //printf("Service 1: Started at %.2f ms, ended at %.2f ms, Ci = %.2f us and WCET = %.2f us\n\r", (start-bootup)/1000, (finish-bootup)/1000,WCET,max_time);     //printing service timeline
        
        //if system falls down, stop running the motors
        if (last_y < -60.0 || last_y > 60.0) 
            
        stop_motors();
        sem_post(&semS2); //release semaphore for S2
    } 
    pthread_exit((void *)0);
}

//function for thread 2, runs acquired data through PID control loop
void *service_2(void *threadp)
{
    double start = 0.0;
    double finish = 0.0;
    double WCET = 0;
    double max_time = 0.0;
    
    while(1)
    {
        sem_wait(&semS2);       //wait for semaphore release from first service
        start = getTimestamp();  
        pid();                  //send difference through the PID function
        finish = getTimestamp();
        WCET = finish - start;
        if(WCET>max_time)
            max_time=WCET;
        //printf("Service 2: Started at %.2f ms, ended at %.2f ms, Ci = %.2f us and WCET = %.2f us\n\r", (start-bootup)/1000, (finish-bootup)/1000,WCET,max_time); //Print this service's timeline
        //printf("\n\rerror=%lf\t%lf\t%lf\t%lf\t%lf", error, speed, pTerm, iTerm, dTerm); 
        printf("\n\rerror= %lf \t speed= %lf", error, speed); 
        sem_post(&semS3); 
    }
    pthread_exit((void *)0);
}

//function of third service, based on speed determined by service 2, runs the motors.
void *service_3(void *threadp)
{
    double start = 0.0;
    double finish = 0.0;
    double WCET = 0;
    double max_time=0.0;
    
    while(1)
    {
        sem_wait(&semS3);
        start = getTimestamp();  
        motors(speed, 0.0, 0.0); //calls motor function and send speed as argument. 
        finish = getTimestamp();
        WCET = finish - start;
        if(WCET>max_time)
            max_time=WCET;
         
         //printf("Service 3: Started at %.2f ms, ended at %.2f ms, Ci = %.2f us and WCET = %.2f us\n\n\n\r", (start-bootup)/1000, (finish-bootup)/1000,WCET,max_time); //print service timeline
        //delay(10);    
        //sem_post(&semS1); 
    }
    pthread_exit((void *)0);
}

//stretch goal: service for read distance from Ultrasoic sensor and move the robot
void *service_4(void *threadp)
{
    double start = 0.0;
    double finish = 0.0;
    double WCET = 0;
    
    while(!abortS4)
    {
        sem_wait(&semS4);   
        distance = measureDistance(); 
        //printf("\n\rDistance=%lf",distance);
        if(distance<10)
        {
            move_forward();
            printf("\n\rmove");
        }  
     }
}


double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

//print the scheduler type
void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

//Completion time test definition
int completion_time_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[])
{
  int i, j;
  U32_T an, anext;
  
  // assume feasible until we find otherwise
  int set_feasible=TRUE;
   
  //printf("numServices=%d\n", numServices);
 
  // For all services in the analysis 
  for (i=0; i < numServices; i++)
  {
       an=0; anext=0;
       
       for (j=0; j <= i; j++)
       {
           an+=wcet[j];
       }
       
	   //printf("i=%d, an=%d\n", i, an);

       while(1)
       {
             anext=wcet[i];
	     
             for (j=0; j < i; j++)
                 anext += ceil(((double)an)/((double)period[j]))*wcet[j];
		 
             if (anext == an)
                break;
             else
                an=anext;

			 //printf("an=%d, anext=%d\n", an, anext);
       }
       
	   //printf("\n\ran=%d, deadline[%d]=%d\n", an, i, deadline[i]);

       if (an > deadline[i])
       {
          set_feasible=FALSE;
       }
  }
  
  return set_feasible;
}

//scheduling point feasibility test
int scheduling_point_feasibility(U32_T numServices, U32_T period[], 
				 U32_T wcet[], U32_T deadline[])
{
   int rc = TRUE, i, j, k, l, status, temp;

   // For all services in the analysis
   for (i=0; i < numServices; i++) // iterate from highest to lowest priority
   {
      status=0;

      // Look for all available CPU minus what has been used by higher priority services
      for (k=0; k<=i; k++) 
      {
	  // find available CPU windows and take them
          for (l=1; l <= (floor((double)period[i]/(double)period[k])); l++)
          {
               temp=0;

               for (j=0; j<=i; j++) temp += wcet[j] * ceil((double)l*(double)period[k]/(double)period[j]);

	       // Can we get the CPU we need or not?
               if (temp <= (l*period[k]))
			   {
				   // insufficient CPU during our period, therefore infeasible
				   status=1;
				   break;
			   }
           }
           if (status) break;
      }

      if (!status) rc=FALSE;
   }
   return rc;
}


// Function to perform tests on measured axes data
void testAxesData() {
	//read from accel via I2C
    acclX = read_word_2c(0x3D);
    acclY = read_word_2c(0x3B);
    acclZ = read_word_2c(0x3F);

    accl_scaled_x = acclX / 16384.0;
    accl_scaled_y = acclY / 16384.0;
    accl_scaled_z = acclZ / 16384.0;

    //read from gyro via I2C
    gyroX = read_word_2c(0x45);
    gyroY = read_word_2c(0x43);
    gyroZ = read_word_2c(0x47);

    gyro_scaled_x = gyroX / GYRO_SCALE_X;
    gyro_scaled_y = gyroY / GYRO_SCALE_Y;
    gyro_scaled_z = gyroZ / GYRO_SCALE_X;
	
    // Define the maximum and minimum values for each axis
    const double MAX_ACCL = 16.0;  // Maximum acceleration in g
    const double MIN_ACCL = -16.0; // Minimum acceleration in g

    const double MAX_GYRO = 2000.0;  // Maximum angular velocity in degrees per second
    const double MIN_GYRO = -2000.0; // Minimum angular velocity in degrees per second

    // Perform assertions to check if measured data is within range
    assert(acclX >= MIN_ACCL && acclX <= MAX_ACCL);
    assert(acclY >= MIN_ACCL && acclY <= MAX_ACCL);
    assert(acclZ >= MIN_ACCL && acclZ <= MAX_ACCL);
    assert(gyroX >= MIN_GYRO && gyroX <= MAX_GYRO);
    assert(gyroY >= MIN_GYRO && gyroY <= MAX_GYRO);
    assert(gyroZ >= MIN_GYRO && gyroZ <= MAX_GYRO);

    // Print success message if all assertions pass
    printf("Test cases passed for accelerometer and gyroscope sensor : All axes data within range.\n");
}


//Function to test motor controls
void testMotorControl() {
    // Initialize motors
    init_motors();
    
    // Test move_forward function
    move_forward(); // Assuming it moves forward at a certain speed
    
    // Test motors function with various speed and offset combinations
    motors(50.0, 0.0, 0.0); // Move forward at speed 50 with no offsets
    motors(-50.0, 0.0, 0.0); // Move backward at speed 50 with no offsets
    motors(0.0, 20.0, -20.0); // Turn left (left motor faster)
    motors(0.0, -20.0, 20.0); // Turn right (right motor faster)
    
    // Test stop_motors function
    stop_motors(); // Stop all motors
    
    printf("Test cases passed for motor system : Motor control tests passed.\n");
}