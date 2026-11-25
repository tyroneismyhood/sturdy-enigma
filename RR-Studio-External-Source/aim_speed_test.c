/* Pure math tests; no input, driver or game access. */
#include "rr_aim.h"
#include "menu_bridge.h"
#include "rr_pose_gate.h"
#include "rr_cham_coherence.h"
#include "rr_frame_pacing.h"
#include "rr_scene_guard.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/* Deterministic synthetic projection; never samples a game or generates input. */
int game_w2s(const RR_CAMERA* camera, const RR_V3* point, int width, int height, float* x, float* y) {
    (void)camera;
    if (!isfinite(point->x) || !isfinite(point->y) || !isfinite(point->z)) return 0;
    *x=width*.5f+point->x; *y=height*.5f-point->y; return 1;
}
int main(void) {
    int failures=0,x,y,fast,slow;
    RR_AIM_STEER state={0};
#define CHECK(c,label) do { int ok=!!(c); printf("[%s] %s\n",ok?"PASS":"FAIL",label); failures+=!ok; } while(0)
    rr_aim_step(&state,100,-50,60,1080,0,100,1,0,&x,&y);
    CHECK(abs(x-100)<=1 && abs(y+50)<=1,"zero smoothing directly corrects projected error");
    state=(RR_AIM_STEER){0}; rr_aim_step(&state,100,0,60,1080,1,100,1,0,&x,&y); fast=x;
    state=(RR_AIM_STEER){0}; rr_aim_step(&state,100,0,60,1080,10,100,1,0,&x,&y); slow=x;
    CHECK(fast>slow && fast>4,"low smoothing is faster than high and previous minimum");
    state=(RR_AIM_STEER){0}; rr_aim_step(&state,100000,-100000,60,1080,0,100,1,0,&x,&y);
    CHECK(x==512 && y==-512,"direct mode remains bounded for large errors");
    rr_aim_step(&state,NAN,0,60,1080,0,100,1,0,&x,&y);
    CHECK(!x&&!y&&!state.carry_x&&!state.carry_y,"non-finite error clears steering");
    rr_aim_step(&state,100,50,60,1080,0,100,0,1,&x,&y);
    CHECK(!x&&!y,"zero elapsed time generates no movement");
    rr_aim_step(&state,.2f,-.2f,60,1080,0,100,1,0,&x,&y);
    CHECK(!x&&!y,"subpixel deadzone prevents jitter");
    state=(RR_AIM_STEER){0}; rr_aim_step(&state,100,-50,60,1080,0,200,1,0,&x,&y);
    CHECK(abs(x-200)<=1&&abs(y+100)<=1,"direct mode respects sensitivity calibration");
    rr_aim_step(&state,100,50,60,1080,0,2147483647,1,0,&x,&y);
    CHECK(!x&&!y,"out-of-range calibration rejected");
    {
        static RR_PLAYER players[2];
        RR_CAMERA camera={0}; BR_Config config={0}; RR_V3 point;
        camera.valid=1; config.aa_fov_px=50; config.aa_bone=RR_AIM_HEAD;
        config.aa_ignore_knocked=1;
        players[0].addr=1; players[0].id=1; players[0].valid=1;
        players[0].health=-1;
        players[0].inveh=1; players[0].pos=(RR_V3){5,0,10};
        CHECK(!rr_aim_point(&players[0],RR_AIM_HEAD,&camera,1920,1080,&point),
              "seated standing-height estimate is not an aim point");
        players[0].mesh_bones=1; players[0].nbones=1; players[0].bone_group[0]=0;
        players[0].bone[0][0]=5; players[0].bone[0][1]=1; players[0].bone[0][2]=10;
        CHECK(rr_aim_point(&players[0],RR_AIM_HEAD,&camera,1920,1080,&point) && point.y==1,
              "live measured seated joint remains targetable");
        players[0].pose_reused=1;
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player,
              "cached seated pose cannot steer aim behind a moving vehicle");
        players[0].pose_reused=0; players[0].bone_group[0]=-1;
        CHECK(!rr_aim_point(&players[0],RR_AIM_HEAD,&camera,1920,1080,&point),
              "missing seated joint never falls back to estimated standing height");
        players[0].bone_group[0]=0;
        CHECK(rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player==&players[0],
              "held valid seated target remains locked");
        players[0].dead=1;
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "down/dead flag releases an existing lock");
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player,
              "down/dead target cannot be reacquired");
        players[0].dead=0; players[0].health=0;
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "confirmed zero health releases an existing lock");
        players[0].health=100;
        players[0].down=1;
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "knockout timestamp state releases a healthy held target");
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player,
               "knocked target is not acquired by the shared aim picker");
        config.aa_ignore_knocked=0;
        CHECK(rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player==&players[0],
              "knocked target is eligible when its independent filter is off");
        config.aa_ignore_knocked=1;
        players[0].down=0;
        CHECK(rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "revived healthy target becomes eligible again");
        strcpy_s(players[0].role, sizeof(players[0].role), "hubber");
        config.aa_ignore_civilian=1;
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "civilian role releases a locked target");
        config.aa_ignore_civilian=0;
        CHECK(rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player==&players[0],
              "civilian remains targetable when filter is off");
        config.aa_ignore_civilian=1;
        strcpy_s(players[0].role, sizeof(players[0].role), "citizen");
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player &&
              !rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "citizen cannot be acquired or kept while civilian filtering is enabled");
        strcpy_s(players[0].role, sizeof(players[0].role), "unknown");
        CHECK(rr_aim_pick(players,1,&camera,1920,1080,&config,0,0,NULL).player==&players[0],
              "unknown role is not mistaken for civilian");
        config.aa_ignore_civilian=0;
        players[0].bone[0][0]=90;
        CHECK(!rr_aim_pick(players,1,&camera,1920,1080,&config,1,1,NULL).player,
              "held lock releases beyond bounded FOV hysteresis");
        players[1]=players[0];players[1].addr=2;players[1].id=2;players[1].bone[0][0]=10;
        CHECK(rr_aim_pick(players,2,&camera,1920,1080,&config,0,0,NULL).player==&players[1],
              "new valid target can be acquired after invalid lock");
        br_friend_set(&config,2,2,1);
        CHECK(br_friend_excluded(&config,2,2),"explicit friend matched without clan metadata");
        CHECK(!rr_aim_pick(players,2,&camera,1920,1080,&config,0,0,NULL).player,
              "explicit friend excluded from new aim acquisition");
        CHECK(!rr_aim_pick(players,2,&camera,1920,1080,&config,2,2,NULL).player,
              "explicit friend excluded even while previously locked");
        CHECK(!br_friend_excluded(&config,2,999),"recycled ID alone cannot exclude a different object");
        br_friends_clear(&config);
        CHECK(rr_aim_pick(players,2,&camera,1920,1080,&config,0,0,NULL).player==&players[1],
              "clearing friend exclusion restores normal selection");
    }
    {
        RR_POSE_GATE gate={0}; float q[4][4]={{0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1}};
        float tip[3]={0};
        CHECK(!rr_pose_gate_hold(&gate,q,4,tip,100,0),"initial valid pose is immediate");
        tip[0]=1;
        CHECK(rr_pose_gate_hold(&gate,q,4,tip,101,0),"abrupt pose is briefly confirmed");
        CHECK(!rr_pose_gate_hold(&gate,q,4,tip,117,0),"stable pose accepted after 16ms instead of 48ms");
        tip[0]=2;rr_pose_gate_hold(&gate,q,4,tip,200,0);
        tip[0]=3;rr_pose_gate_hold(&gate,q,4,tip,210,0);
        tip[0]=4;
        CHECK(!rr_pose_gate_hold(&gate,q,4,tip,224,0),"continuous changes cannot hold skeleton beyond 24ms");
    }
    {
        static float old_pose[RR_CHAM_BONES][12], new_pose[RR_CHAM_BONES][12];
        unsigned since=0; int pending=0;
        new_pose[RR_RIG_M_HEAD_1_JNT][0]=1;
        new_pose[RR_RIG_L_ARM_1_JNT][0]=1;
        new_pose[RR_RIG_R_ARM_1_JNT][0]=1;
        CHECK(rr_cham_coherence_hold(old_pose,new_pose,0,0,100,1,&since,&pending),
              "coordinated discontinuity is briefly held");
        CHECK(!rr_cham_coherence_hold(old_pose,new_pose,0,0,116,17,&since,&pending),
              "coherence hold releases after 16ms instead of 40ms");
    }
    {
        uint64_t next=0, period=rr_frame_period(1000000,240); int frames=0;
        for (uint64_t now=0;now<1000000;now+=1000) {
            if (now>=next) { ++frames;next=rr_frame_advance(now,next,period); }
        }
        CHECK(period==4167,"240 FPS uses 4.167ms period instead of integer 4/5ms");
        CHECK(frames==240,"deadline pacing targets 240 updates with 1ms worker polling");
        CHECK(rr_frame_advance(100000,4167,4167)==104167,"long stall resets deadline without catch-up burst");
        CHECK(rr_frame_period(1000000,5)==200000,"empty overlay retains low-load idle pacing");
    }
    {
        BR_Config friends={0};
        CHECK(br_friend_set(&friends,1,101,1)&&br_friend_set(&friends,2,202,1)&&
              br_friend_excluded(&friends,1,101)&&br_friend_excluded(&friends,2,202),
              "multiple friends excluded simultaneously");
        CHECK(br_friend_set(&friends,1,0,0)&&!br_friend_excluded(&friends,1,101)&&
              br_friend_excluded(&friends,2,202),"removing one friend preserves other exclusions");
        CHECK(br_friend_set(&friends,2,222,1)&&!br_friend_excluded(&friends,2,202)&&
              br_friend_excluded(&friends,2,222),"explicit reselection updates object identity");
        br_friends_clear(&friends);
        int full=1;
        for(int i=0;i<BR_MAX_FRIENDS;++i) full &= br_friend_set(&friends,i,1000+i,1);
        CHECK(full&&!br_friend_set(&friends,999,9999,1),"supports 320 friends and rejects overflow safely");
        CHECK(br_friend_set(&friends,2,0,0)&&br_friend_set(&friends,999,9999,1)&&
              br_friend_excluded(&friends,999,9999),"removed slots can be reused");
        br_friends_clear(&friends);int empty=1;
        for(int i=0;i<BR_MAX_FRIENDS;++i) empty &= friends.friends[i].address==0;
        CHECK(empty,"clear all removes every exclusion");
    }
    {
        RR_SCENE_GUARD scene={0};
        CHECK(!rr_scene_observe(&scene,10,100,1),"first scene sample waits for confirmation");
        CHECK(rr_scene_observe(&scene,10,100,1),"second coherent scene sample becomes ready");
        unsigned epoch=scene.epoch;
        CHECK(!rr_scene_observe(&scene,20,200,1)&&scene.epoch!=epoch,
              "map and camera change invalidates old scene epoch");
        CHECK(rr_scene_observe(&scene,20,200,1),"new map becomes ready only after coherent confirmation");
        CHECK(!rr_scene_observe(&scene,20,200,0),"world change during a sample cannot publish mixed data");
        CHECK(!rr_scene_observe(&scene,20,200,1)&&rr_scene_observe(&scene,20,200,1),
              "incoherent read requires fresh confirmation before recovery");
        CHECK(!rr_scene_observe(&scene,20,0,1),"estimated or missing camera cannot authorize world-space ESP");
        epoch=scene.epoch;
        CHECK(!rr_scene_observe(&scene,20,300,1)&&scene.epoch!=epoch,
              "camera replacement alone invalidates cached visual poses");
        CHECK(rr_scene_observe(&scene,20,300,1),"camera replacement recovers after confirmation");
        CHECK(!rr_scene_observe(&scene,0,0,0),"loading screen with no world is not drawable");
    }
    return failures?1:0;
}
