/*****************************************************************************************/
//							    		プログラム名									 //
/*****************************************************************************************/
//
//
//
//
/*****************************************************************************************/



//=======================================================================================//
//									インクルードヘッダ
//=======================================================================================//
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
//#include "texturepath.h"


//=======================================================================================//
//									変数関数の定義宣言
//=======================================================================================//

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup; // コンタクトグループ
dReal r = 0.2, m  = 1.0;
dsFunctions fn;

typedef struct {       // MyObject構造体
  dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
  double  l,r,x,y,z,m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

static MyObject obj[5];    // leg[0]:上脚, leg[1]:下脚
static dJointID h_joint,s_joint; // ヒンジ, スライダー
static int STEPS = 0;            // シミュレーションのステップ数
static dReal S_LENGTH = 0.0;     // スライダー長
static dReal H_ANGLE  = 0.0;     // ヒンジ角



//=======================================================================================//
//										衝突検出
//=======================================================================================//
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 7;     // 接触点数
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  // 2つのボディがジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode   = dContactBounce | dContactSoftERP |
                                  dContactSoftCFM;
      contact[i].surface.soft_erp   = 0.1;   // 接触点のERP
      contact[i].surface.soft_cfm   = 0.001; // 接触点のCFM
      contact[i].surface.mu     = dInfinity; // 摩擦係数:無限大
      dJointID c = dJointCreateContact(world,
                                       contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}



// ヒンジジョイントの制御
static void controlHinge(dReal target)
{
  static dReal kp = 10.0, fmax = 1000;

  dReal tmp   = dJointGetHingeAngle(h_joint);
  dReal diff  = target - tmp;
  if (diff >=   M_PI) diff -= 2.0 * M_PI; // diffが2πより小さく
  if (diff <= - M_PI) diff += 2.0 * M_PI; // diffが-2πより大きく
  dReal u     = kp * diff;

  dJointSetHingeParam(h_joint, dParamVel,  u);
  dJointSetHingeParam(h_joint, dParamFMax, fmax);
}

// スライダージョイントの制御
static void controlSlider(dReal target)
{
  static dReal max_force = 1000;

  if (target > 0) dJointSetSliderParam(s_joint, dParamVel,  8.0);
  else            dJointSetSliderParam(s_joint, dParamVel, -8.0);
  dJointSetSliderParam(s_joint, dParamFMax, max_force);
}


//=======================================================================================//
//									ロボットの描画
//=======================================================================================//
static void drawRobot()
{
  //球形の描画
  dsSetColor(1.2,0.0,0.0); 
  dsDrawSphere(dBodyGetPosition(obj[1].body),dBodyGetRotation(obj[0].body),obj[0].r);
  //円筒の描画
  dsSetColor(1.2,1.0,0.0); 
  dsDrawCylinder(dBodyGetPosition(obj[0].body),dBodyGetRotation(obj[1].body),obj[1].l,obj[1].r);
  //カプセルの描画
  dsSetColor(1.2,0.0,1.0); 
  dsDrawCapsule(dBodyGetPosition(obj[2].body),dBodyGetRotation(obj[2].body),obj[2].l,obj[2].r);
  //長方形の描画
  dsSetColor(0.2,0.0,0.0); 
  dVector3 sides;
  dGeomBoxGetLengths(obj[3].geom,sides);
  dsDrawBox(dBodyGetPosition(obj[3].body),dBodyGetRotation(obj[3].body),sides);


}

//=======================================================================================//
//									シミュレーションループ
//=======================================================================================//
static void simLoop(int pause)
{

  if (!pause) {
    STEPS++;
    dSpaceCollide(space,0,&nearCallback);
    dWorldStep(world,0.01);
    dJointGroupEmpty(contactgroup);
  }

	drawRobot();

}

//=======================================================================================//
//									ロボットの生成
//=======================================================================================//
void createRobot()
{
	dMass mass;
	dReal x0=0,y0=0,z0=0;
	dReal x=0,y=0,z=0;
	dReal ax=1,ay=0,az=0;
	dReal angle=0*M_PI/180;
	dMatrix3 R;

//球体の生成
  obj[0].r    = 0.25;
  obj[0].m    = 14.0;
  obj[0].body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass,obj[0].m,obj[0].r);
  dBodySetMass(obj[0].body,&mass);
  dBodySetPosition(obj[0].body, x0, y0-2, z0+1);

  obj[0].geom = dCreateSphere(space,obj[0].r);
  dGeomSetBody(obj[0].geom,obj[0].body);


//円筒の生成
  obj[1].r=0.1,obj[1].l=0.5,obj[1].m=1;

  obj[1].body =dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass,obj[1].m,3,obj[1].r,obj[1].l);
  dBodySetMass(obj[1].body,&mass);
  dBodySetPosition(obj[1].body,x0,y0-0.7,z0+1);

  obj[1].geom = dCreateCylinder(space,obj[1].r,obj[1].l);
  dGeomSetBody(obj[1].geom,obj[1].body);

  dRFromAxisAndAngle(R,ax,ay,az,angle);
  dBodySetRotation(obj[1].body,R);


//カプセルの生成
  obj[2].r=0.1,obj[2].l=0.5,obj[2].m=1;

  obj[2].body =dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,obj[2].m,3,obj[2].r,obj[2].l);
  dBodySetMass(obj[2].body,&mass);
  dBodySetPosition(obj[2].body,x0,y0+0.7,z0+1);

  obj[2].geom = dCreateCapsule(space,obj[2].r,obj[2].l);
  dGeomSetBody(obj[2].geom,obj[2].body);

  dRFromAxisAndAngle(R,ax,ay,az,angle);
  dBodySetRotation(obj[2].body,R);


//長方形の生成
  obj[3].x=0.3,obj[3].y=0.4,obj[3].z=0.5,obj[3].m=1;

  obj[3].body =dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass,obj[3].m,obj[3].x,obj[3].y,obj[3].z);
  dBodySetMass(obj[3].body,&mass);
  dBodySetPosition(obj[3].body,x0,y0+2,z0+1);

  obj[3].geom = dCreateBox(space,obj[3].x,obj[3].y,obj[3].z);
  dGeomSetBody(obj[3].geom,obj[3].body);

  dRFromAxisAndAngle(R,ax,ay,az,angle);
  dBodySetRotation(obj[3].body,R);




}

//=======================================================================================//
//									ロボットの破壊
//=======================================================================================//
void destroyRobot()
{

  for (int i = 0; i < 4; i++) {
    dBodyDestroy(obj[i].body);
    dGeomDestroy(obj[i].geom); 
  }

}

//=======================================================================================//
//									リスタート関数
//=======================================================================================//
static void restart()
{
  STEPS    = 0;      // ステップ数の初期化
  S_LENGTH = 0.0;    // スライダ長の初期化
  H_ANGLE  = 0.0;    // ヒンジ角度の初期化

  destroyRobot();  // ロボットの破壊
  dJointGroupDestroy(contactgroup);     // ジョイントグループの破壊
  contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
  createRobot();				          // ロボットの生成
}

//=======================================================================================//
//										キー操作
//=======================================================================================//
static void command(int cmd)
{
 switch (cmd) {
   case 'j':S_LENGTH =   0.25; break;
   case 'f':S_LENGTH = - 0.25; break;
   case 'k':H_ANGLE +=   0.25; break;
   case 'd':H_ANGLE -=   0.25; break;
   case 'r':restart()                           ; break;
   default :printf("key missed \n")             ; break;
 }
}

//=======================================================================================//
//									スタート関数
//=======================================================================================//
static void start()
{
  static float xyz[3] = {   3.5, 0.0, 1.0};
  static float hpr[3] = {-180.0, 0.0, 0.0};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

void setDrawStuff()           /*** 描画関数の設定 ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.command = &command;      // キー入力関数へのポインタ
  fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}

//=======================================================================================//
//									メイン関数
//=======================================================================================//
int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();

  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world, 0,0, -9.8);
  dWorldSetERP(world, 0.9);          // ERPの設定
  dWorldSetCFM(world, 1e-4);         // CFMの設定
  ground = dCreatePlane(space, 0, 0, 1, 0);
  createRobot();

  dBodyDisable(obj[0].body);
  dBodyDisable(obj[1].body);
  dBodyDisable(obj[2].body);
  dBodyDisable(obj[3].body);
  dsSimulationLoop (argc, argv, 640, 480, &fn);
  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
