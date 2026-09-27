/* Controlled test inputs only; no planner or safety equivalence claim. */
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"
#include "xgc_dmpc_planner_v1.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
_Static_assert(sizeof(xgc_position_target_v1)==104,"PositionTarget ABI");
_Static_assert(offsetof(xgc_position_target_v1,acceleration)==56,"acceleration offset");
_Static_assert(offsetof(xgc_position_target_v1,type_mask)==96,"mask offset");
_Static_assert(sizeof(xgc_dmpc_paired_state_v1)==96,"paired ABI");
_Static_assert(offsetof(xgc_dmpc_paired_state_v1,linear_velocity)==72,"velocity offset");
_Static_assert(sizeof(xgc_controller_status_v1)==56,"status ABI");
static const xgc_host_api* h;
static FILE* out;
static int prepared,started,stopped;
static double effective;
static double last_stamp,last_position,last_velocity;
static void* create(const xgc_host_api* api){h=api;return &prepared;}
static xgc_status configure(void* p,const char* text){(void)p;(void)text;out=fopen(OUTPUT,"w");return out?XGC_OK:XGC_ERR;}
static xgc_status life(void* p){(void)p;return XGC_OK;}
static void send_command(const char* text,const xgc_step_ctx* c){xgc_command_v1 m={0};snprintf(m.text,64,"%s",text);h->publish(h->host,0,c->round,(const uint8_t*)&m,sizeof m);}
static xgc_status step(void* p,const xgc_step_ctx* c){
 (void)p;xgc_sample_view s;int ready=0,configured=0;
 while(h->next(h->host,2,&s)==XGC_OK){
  if(s.len!=sizeof(xgc_dmpc_paired_state_v1))return XGC_ERR;
  xgc_dmpc_paired_state_v1 m;memcpy(&m,s.data,sizeof m);
  if(m.pose_stamp_sec!=m.twist_stamp_sec || m.orientation_xyzw[3]!=1.0)return XGC_ERR;
  last_stamp=m.pose_stamp_sec;last_position=m.position[0];last_velocity=m.linear_velocity[0];
  fprintf(out,"P %.17g %.17g %.17g\n",m.pose_stamp_sec,m.position[0],m.linear_velocity[0]);
 }
 while(h->next(h->host,3,&s)==XGC_OK){
  if(s.len!=sizeof(xgc_controller_status_v1))return XGC_ERR;
  xgc_controller_status_v1 m;memcpy(&m,s.data,sizeof m);
  fprintf(out,"S %.17g %s\n",m.stamp,m.state);if(strcmp(m.state,"Ready")==0)ready=1;if(strcmp(m.state,"Configured")==0)configured=1;
 }
 if(configured&&!prepared){send_command("prepare",c);prepared=1;}
 if(ready&&!started){
  send_command("custom1",c);
  xgc_position_target_v1 m={0};effective=(double)c->now*1e-9+0.1;m.stamp=effective;
  m.position[0]=last_position+last_velocity*(effective-last_stamp);m.position[1]=2;m.position[2]=3;
  m.velocity[0]=last_velocity;
  m.acceleration[0]=2;m.coordinate_frame=1;
  fprintf(out,"E %.17g\n",effective);
  h->publish(h->host,1,c->round,(const uint8_t*)&m,sizeof m);started=1;
 }
 if(started&&!stopped&&(double)c->now*1e-9>effective+0.15){send_command("stop",c);stopped=1;}
 fflush(out);return XGC_OK;
}
static void destroy(void* p){(void)p;if(out){fclose(out);out=0;}}
static const char* state(void* p){(void)p;return "test-wire-driver";}
static const xgc_port_decl ports[]={
 {"command",XGC_PORT_OUT,"xgc.command/1",XGC_QOS_EVENT},
 {"position_target",XGC_PORT_OUT,"xgc.position_target/1",XGC_QOS_CONTROL},
 {"paired_state",XGC_PORT_IN,"xgc.dmpc.paired_state/1",XGC_QOS_STATE},
 {"controller_state",XGC_PORT_IN,"xgc.controller_status/1",XGC_QOS_STATE}};
static const xgc_plugin_vtbl vt={create,configure,life,step,life,destroy,state};
static const xgc_plugin_descriptor desc={1,4,"numeric-test-wire","1",ports,&vt};
const xgc_plugin_descriptor* xgc_rt_plugin_v1(void){return &desc;}
