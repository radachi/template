/*****************************************************************************************/
// プログラム名 安達がいじった
/*****************************************************************************************/
// 説明文
//
//
//
/*****************************************************************************************/



//=======================================================================================//
// インクルードヘッダ
//=======================================================================================//
#include <ode/ode.h>				// ODEライブラリのインクルード
#include <drawstuff/drawstuff.h>	// drawstuffライブラリのインクルード

//=======================================================================================//
// 変数関数の定義宣言
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

dWorldID world;						// 動力学計算用ワールド
dSpaceID space;						// 衝突検出用スペース
dGeomID  ground;					// 地面
dJointGroupID contactgroup;			// コンタクトグループ
dsFunctions fn;

typedef struct {					// MyObject構造体
	dBodyID body;					// ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;					// ジオメトリのID番号(衝突検出計算用）
	double  l,r,x,y,z,m;			// 長さl[m], 半径r[m]，辺x,y,z[m], 質量m[kg]
} MyObject;

// MyObjectのインスタンス
static MyObject sSphere, sCylinder, sCapsule, sBox;

static int STEPS = 0;				// シミュレーションのステップ数

//=======================================================================================//
// 衝突検出
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
			contact[i].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
			contact[i].surface.soft_erp = 0.1;			// 接触点のERP
			contact[i].surface.soft_cfm = 0.001;		// 接触点のCFM
			contact[i].surface.mu       = dInfinity;	// 摩擦係数:無限大
			dJointID c = dJointCreateContact(world, contactgroup,&contact[i]);
			dJointAttach(c,dGeomGetBody(contact[i].geom.g1),
			dGeomGetBody(contact[i].geom.g2));
		}
	}
}

//=======================================================================================//
// 物体の生成
//=======================================================================================//
void createObject()
{
	dMass mass;
	dReal x0=0,y0=0,z0=0;		// 基準点
	dReal ax=1,ay=0,az=0;		// 回転軸（ｘ軸周り）
	dReal angle=0*M_PI/180;		// 回転角度
	dMatrix3 R;

	// 球体の生成
	sSphere.r    = 0.25;
	sSphere.m    = 14.0;
	sSphere.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass,sSphere.m,sSphere.r);
	dBodySetMass(sSphere.body,&mass);
	dBodySetPosition(sSphere.body, x0, y0-2, z0+1);

	sSphere.geom = dCreateSphere(space,sSphere.r);
	dGeomSetBody(sSphere.geom,sSphere.body);


	// 円筒の生成
	sCylinder.r=0.1,sCylinder.l=0.5,sCylinder.m=1;

	sCylinder.body =dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass,sCylinder.m,3,sCylinder.r,sCylinder.l);
	dBodySetMass(sCylinder.body,&mass);
	dBodySetPosition(sCylinder.body,x0,y0-0.7,z0+1);

	sCylinder.geom = dCreateCylinder(space,sCylinder.r,sCylinder.l);
	dGeomSetBody(sCylinder.geom,sCylinder.body);

	dRFromAxisAndAngle(R,ax,ay,az,angle);
	dBodySetRotation(sCylinder.body,R);


	// カプセルの生成
	sCapsule.r=0.1,sCapsule.l=0.5,sCapsule.m=1;

	sCapsule.body =dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass,sCapsule.m,3,sCapsule.r,sCapsule.l);
	dBodySetMass(sCapsule.body,&mass);
	dBodySetPosition(sCapsule.body,x0,y0+0.7,z0+1);

	sCapsule.geom = dCreateCapsule(space,sCapsule.r,sCapsule.l);
	dGeomSetBody(sCapsule.geom,sCapsule.body);

	dRFromAxisAndAngle(R,ax,ay,az,angle);
	dBodySetRotation(sCapsule.body,R);


	// ボックスの生成
	sBox.x=0.3,sBox.y=0.4,sBox.z=0.5,sBox.m=1;

	sBox.body =dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass,sBox.m,sBox.x,sBox.y,sBox.z);
	dBodySetMass(sBox.body,&mass);
	dBodySetPosition(sBox.body,x0,y0+2,z0+1);

	sBox.geom = dCreateBox(space,sBox.x,sBox.y,sBox.z);
	dGeomSetBody(sBox.geom,sBox.body);

	dRFromAxisAndAngle(R,ax,ay,az,angle);
	dBodySetRotation(sBox.body,R);

}


//=======================================================================================//
// 物体の描画
//=======================================================================================//
static void drawObject()
{
	//球形の描画
	dsSetColor(1.2,0.0,0.0); 
	dsDrawSphere(dBodyGetPosition(sSphere.body),dBodyGetRotation(sSphere.body),sSphere.r);

	//円筒の描画
	dsSetColor(1.2,1.0,0.0); 
	dsDrawCylinder(dBodyGetPosition(sCylinder.body),dBodyGetRotation(sCylinder.body),sCylinder.l,sCylinder.r);

	//カプセルの描画
	dsSetColor(1.2,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(sCapsule.body),dBodyGetRotation(sCapsule.body),sCapsule.l,sCapsule.r);

	//ボックスの描画
	dsSetColor(0.2,0.0,0.0); 
	dVector3 sides;
	dGeomBoxGetLengths(sBox.geom,sides);
	dsDrawBox(dBodyGetPosition(sBox.body),dBodyGetRotation(sBox.body),sides);
}

//=======================================================================================//
// 物体の破壊
//=======================================================================================//
void destroyObject()
{
	// 球体の破壊
	dBodyDestroy(sSphere.body);
	dGeomDestroy(sSphere.geom);

	// 円筒の破壊
	dBodyDestroy(sCylinder.body);
	dGeomDestroy(sCylinder.geom);

	// カプセルの破壊
	dBodyDestroy(sCapsule.body);
	dGeomDestroy(sCapsule.geom);

	// ボックスの破壊
	dBodyDestroy(sBox.body);
	dGeomDestroy(sBox.geom);
}

//=======================================================================================//
// シミュレーションループ
//=======================================================================================//
static void simLoop(int pause)
{

	if (!pause) {
		STEPS++;
		dSpaceCollide(space,0,&nearCallback);
		dWorldStep(world,0.01);
		dJointGroupEmpty(contactgroup);
	}

	// 物体の描画
	drawObject();

}

//=======================================================================================//
// リスタート関数
//=======================================================================================//
static void restart()
{
	STEPS    = 0;							// ステップ数の初期化
	destroyObject();						// ロボットの破壊
	dJointGroupDestroy(contactgroup);		// ジョイントグループの破壊
	contactgroup = dJointGroupCreate(0);	// ジョイントグループの生成
	createObject();							// ロボットの生成
}

//=======================================================================================//
// キー操作
//=======================================================================================//
static void command(int cmd)
{
	switch (cmd) {
		case 'r':restart()                           ; break;
		case 'q':exit(0)	                         ; break;
		default :printf("key missed \n")             ; break;
	}
}

//=======================================================================================//
// スタート関数
//=======================================================================================//
static void start()
{
	static float xyz[3] = {   3.5, 0.0, 1.0};
	static float hpr[3] = {-180.0, 0.0, 0.0};
	dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
	dsSetSphereQuality(3);                 // 球の品質設定
}

//=======================================================================================//
// 描画の設定
//=======================================================================================//
void setDrawStuff()
{
	fn.version = DS_VERSION;    // ドロースタッフのバージョン
	fn.start   = &start;        // 前処理 start関数のポインタ
	fn.step    = &simLoop;      // simLoop関数のポインタ
	fn.command = &command;      // キー入力関数へのポインタ
	fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}

//=======================================================================================//
// メイン関数
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
	createObject();

	//dBodyDisable(obj[0].body);
	//dBodyDisable(sCylinder.body);
	//dBodyDisable(sCapsule.body);
	//dBodyDisable(sBox.body);

	dsSimulationLoop (argc, argv, 640, 480, &fn);
	dWorldDestroy (world);
	dCloseODE();

	return 0;
}
