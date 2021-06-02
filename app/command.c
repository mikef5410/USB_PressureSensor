#include "OSandPlatform.h"
#include "debug_shell.h"
#include "ms8607.h"

#define GLOBAL_VERSION
#include "version.h"

#ifndef COUNTOF
#define COUNTOF(A) (sizeof(A)/sizeof(A[0]))
#endif


#ifdef BUILD_INFO
static int cmd_buildInfo(int argc, char **argv)
{
  (void) argc;
  (void) argv;
  myprintf("%s\r\n", build_info);
  return (0);
}
#endif

#ifdef BUILD_SHA1
static int cmd_build_sha1(int argc, char **argv)
{
  (void) argc;
  (void) argv;
  myprintf("%s, %s\r\n", build_sha1, build_sha1_full);
  return (0);
}
#endif

static int cmd_hardfault(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  TRIG_HARDFAULT; //trigger a hard fault
  return(0);
}

static int cmd_timer(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  for (int j=0; j<3; j++) {
    gpio_toggle(GPIOC, GPIO7);
    volatile uint64_t start=hiresTimer_getTime();
    vTaskDelay(10/portTICK_RATE_MS);
    volatile int64_t delta = hiresTimer_getTime() - start;
    //gpio_clear(GPIOC, GPIO7);
    myprintf(" 10 ms = %d us \n", (int)tics2us(delta));
  }
  return(0);
}

#ifdef TESTEEPROM
static int cmd_testee(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  eeprom9366_test();
  return(0);
}
#endif

static int cmd_testthp(int argc, char **argv)
{
  (void) argc;
  (void) argv;
  float t, h, p, hc;
  int tt, hh, pp;
  int rval=0;
  
  if (ms8607_is_connected()) {
    ms8607_reset();
    ms8607_read_temperature_pressure_humidity(&t, &p, &h);
    ms8607_get_compensated_humidity(t,h,&hc);
    tt=(int)t;
    int ttf = (int) (fabs(t-tt)*100);
    pp=(int)p;
    hh=(int)(hc);
    int hhf = (int) (fabs(hc-hh)*100);
    myprintf("T=%d.%02d, P=%d, H=%d.%02d\n",tt,ttf,pp,hh,hhf);
  } else {
    myprintf("No MS8607 Temp/Hum/Press sensor.\n");
    rval=1;
  }
  return(rval);
}


dispatchEntry mainCommands[] = {
//Context, Command,        ShortHelp,                                          command proc,  help proc
#ifdef BUILD_INFO
  {"","buildInfo",        "                      Show build info", cmd_buildInfo, NULL},
#endif
#ifdef BUILD_SHA1
  {"","build_sha1",       "                      Show SHA1 info", cmd_build_sha1, NULL},
#endif
  {"","hardfault",        "                      Cause a hard fault", cmd_hardfault, NULL},
  {"","timer",            "                      Test the hires timer", cmd_timer, NULL},
#ifdef TESTEEPROM
  {"","testee",           "                      Test eeprom", cmd_testee, NULL},
#endif
  {"","testthp",          "                      Test Temp/Hum/Press sensor", cmd_testthp, NULL},
  //LAST ENTRY
  {NULL, NULL, NULL, NULL, NULL}
};


// Add your command table here ... most general last
dispatchEntry* dispatchTableTable[] = {
    &(mainCommands[0]),     //command.c
    &(commonCommands[0]),  //debug_shell.c
    NULL
};
