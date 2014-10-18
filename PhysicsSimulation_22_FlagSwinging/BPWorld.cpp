#include "BPWorld.h"
#include "GlutStuff.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include <iostream>
#include <cstdlib>

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

//#define USE_PROF
#ifdef USE_PROF
#include <fstream>
std::ofstream g_prof("sim_prof.txt");
#endif

//// Begin: 定数の実体
const btScalar BPWorld::G_ACC;
const btScalar BPWorld::ERP;
//// End

////////////////////////////////////////////////////////////////////////////////
/**
 *  コンストラクタ
 */
BPWorld::BPWorld(char* winTitle, int winWidth, int winHeight, MonitoringFunc monFnPtr, KeyboardFunc keyFnPtr)
{
  m_softBodyRequested = false;
  init(winTitle, winWidth, winHeight, monFnPtr, keyFnPtr);
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  コンストラクタ
 */
BPWorld::BPWorld(char* winTitle, int winWidth, int winHeight, WorldType wldType, MonitoringFunc monFnPtr, KeyboardFunc keyFnPtr)
{
  if(wldType == BPWorld::RIGID_BODY_ONLY){
    m_softBodyRequested = false;
  }else if(wldType == BPWorld::SOFT_BODY_INCLUDED){
    m_softBodyRequested = true;
  }
  init(winTitle, winWidth, winHeight, monFnPtr, keyFnPtr);
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  初期化処理（2種類のコンストラクタの共通部分）
 */
void BPWorld::init(char* winTitle, int winWidth, int winHeight, MonitoringFunc monFnPtr, KeyboardFunc keyFnPtr)  
{
  m_broadphase = NULL;
  m_dispatcher = NULL;
  m_solver = NULL;
  m_collisionConfiguration = NULL;
  m_timeStep    = DEFAULT_TIME_STEP;
  m_fixedTimeStepEnabled = false;
  m_currentTime = 0;
  m_winTitle  = winTitle;
  m_winWidth  = winWidth;
  m_winHeight = winHeight;
  m_doMonitoring = monFnPtr;
  m_checkKey = keyFnPtr;
  
  // 動力学シミュレーションの準備
  initPhysics();

  //// 描画設定
  // 描画機能を追加するクラスを使うための呼び出し
  overrideGLShapeDrawer(new BPWorld::NewShapeDrawer());
  setTexturing(true);
  setShadows(true);
  setCameraDistance(20);
  m_dynamicsWorld->setDebugDrawer(&m_debugDrawer);
  setDebugMode(btIDebugDraw::DBG_NoHelpText |         // ヘルプテキスト表示を消す
               0);
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  動力学シミュレーションの準備を行う
 */
void BPWorld::initPhysics()
{
  if(m_softBodyRequested){
    initWorldForSoftBody();
  }else{
    initWorldForRigidOnly();
  }
  m_dynamicsWorld->setGravity(btVector3(0, -G_ACC, 0)); // 重力加速度の設定
  btContactSolverInfo& slvInfo = m_dynamicsWorld->getSolverInfo();
  //slvInfo.m_numIterations = 10;  
  slvInfo.m_erp  = ERP;
  //slvInfo.m_erp2 = 0.0;
  //slvInfo.m_globalCfm = 1.0;
  //slvInfo.m_linearSlop = 0.0;
  //slvInfo.m_warmstartingFactor = 0.85;
  //slvInfo.m_splitImpulse = 0;
  //slvInfo.m_restitution = 1.0;
  //slvInfo.m_restingContactRestitutionThreshold = 2;
  //slvInfo.m_solverMode |= SOLVER_USE_WARMSTARTING;
  //slvInfo.m_solverMode |= SOLVER_RANDMIZE_ORDER;
  
  

  // 地面（ワールド座標系の Z-X 平面）
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
  m_collisionShapes.push_back(groundShape); // 終了処理のときにメモリを解放するためポインタを格納．
                                            // 解放が自動で行われないので必要な処理．

  btTransform transform;
  transform.setIdentity();

  // DemoApplication::localCreateRigidBody()はbtRigidBodyの生成，
  // そのbtRigidBodyのm_dynamicsWorldへの登録を行う．
  // 指定された形状に応じた慣性モーメント行列の計算も行われる．
  btRigidBody *groundBody = localCreateRigidBody(0, // 質量, 0にすると固定された静止物体として扱われる
						 transform,
						 groundShape);
  groundBody->setRestitution(1.0); // 反発係数．初期値が0なので反発してほしい物体全てに明示的に設定する  
  groundBody->setFriction(1.0);

  // 三角メッシュ（btGImpactMeshShape）形状の物体の衝突判定のためには下記が必要
  // （btBoxShape等の基本形状の物体の衝突判定だけならば不要）
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  剛体のみのシミュレーションを行う場合のワールドの設定
 */
void BPWorld::initWorldForRigidOnly()
{
  ///collision configuration contains default setup for memory, collision setup
  m_collisionConfiguration = new btDefaultCollisionConfiguration();

  ///use the default collision dispatcher.
  // For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btDbvtBroadphase();

  ///the default constraint solver.
  // For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  m_solver = new btSequentialImpulseConstraintSolver;
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver,
                                                m_collisionConfiguration);
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  剛体および柔軟体のシミュレーションを行う場合のワールドの設定
 */
void BPWorld::initWorldForSoftBody()
{
  btVector3 worldAabbMin(-1000, -1000, -1000);
  btVector3 worldAabbMax(1000, 1000, 1000);
  const int MAX_PROXIES = 32766;

  m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, MAX_PROXIES);
  m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_solver = new btSequentialImpulseConstraintSolver();

  m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher,
                                                 m_broadphase,
                                                 m_solver,
                                                 m_collisionConfiguration);
  m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
  
  m_softBodyWorldInfo.m_broadphase = m_broadphase;
  m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
  
  m_softBodyWorldInfo.air_density   = 1.2;
  m_softBodyWorldInfo.water_density = 0;
  m_softBodyWorldInfo.water_offset  = 0;
  m_softBodyWorldInfo.water_normal  = btVector3(0,0,0);
  m_softBodyWorldInfo.m_gravity.setValue(0, -G_ACC, 0);

  
  m_softBodyWorldInfo.m_sparsesdf.Initialize();
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  デストラクタ．
 *  確保したメモリ（btCollisionShape, etc.）を解放する
 */
BPWorld::~BPWorld()
{
  // cleanup in the reverse order of creation/initialization
  // remove the rigid bodies from the dynamics world and delete them
  for(int i = m_dynamicsWorld->getNumCollisionObjects()-1; i >= 0; i--){
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()){
      delete body->getMotionState();
    }
    m_dynamicsWorld->removeCollisionObject( obj );
    delete obj;
  }

  // delete collision shapes
  for(int j = 0; j < m_collisionShapes.size(); j++){
    btCollisionShape* shape = m_collisionShapes[j];
    delete shape;
  }

  // delete constraints
  for(int i = m_dynamicsWorld->getNumConstraints()-1; i >= 0; i--){
    btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
    m_dynamicsWorld->removeConstraint(constraint);
    delete constraint;
  }

  delete m_dynamicsWorld;
  delete m_solver;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_collisionConfiguration;  
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  ウィンドウを表示して，シミュレーションを開始する
 */
int BPWorld::start()
{
  int  argCount = 1;
  char *argVec[] = {m_winTitle};
  return glutmain(argCount, argVec, m_winWidth, m_winHeight, m_winTitle, this);
}



////////////////////////////////////////////////////////////////////////////////
/**
 *  物体の力学データを作成し，ワールドに登録する
 *
 *  @param mass 作成する物体の質量
 *  @param initTransform 作成する物体の初期位置・姿勢
 *  @param shape 作成する物体の形状
 */
btRigidBody* BPWorld::createRigidBody(btScalar mass,
                                      const btTransform& initTransform,
                                      btCollisionShape* shape)
{
  // 二重解放を避けるため，まだ登録されていない形状データかどうか
  // 確認してからshapeを配列に追加する．
  // findLinearSearch()で見つからない場合は，返り値は配列のサイズになる
  if(m_collisionShapes.findLinearSearch(shape) == m_collisionShapes.size()){
    m_collisionShapes.push_back(shape); // 終了時に解放するために保存
  }//else{
  //std::cout << "Duplicate element found" << std::endl;
  //}
  

  // 指定された物体を作ってワールドに登録する
  return localCreateRigidBody(mass, initTransform, shape); 
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  ジョイント（constraint）をワールドに登録する
 */
void BPWorld::addConstraint(btTypedConstraint *constraint, 
                            bool disableCollisionsBetweenLinkedBodies)
{
  m_dynamicsWorld->addConstraint(constraint,
                                 disableCollisionsBetweenLinkedBodies);
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  ジョイント（constraint）をワールドから削除する
 */
void BPWorld::removeConstraint(btTypedConstraint *constraint)
{
  m_dynamicsWorld->removeConstraint(constraint);
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  シミュレーションのメインループから呼ばれるメンバ関数（コールバック）
 */
void BPWorld::clientMoveAndDisplay()
{
  if(m_doMonitoring != NULL){
    m_doMonitoring(m_currentTime); // ユーザが定義したモニタリング関数を呼び出す
  }
  
  // シミュレーションを進める 
  if(m_dynamicsWorld != NULL){
    if(m_fixedTimeStepEnabled){
      const int NO_SUBDIVISION = 0;
      m_dynamicsWorld->stepSimulation(m_timeStep, NO_SUBDIVISION);
      m_currentTime += m_timeStep;
    }else{
      const btScalar USEC_TO_SEC = 1.0/(1000*1000);
      btScalar dt = m_clock.getTimeMicroseconds()*USEC_TO_SEC; // 前フレーム描画からの経過時間の測定
      const int MAX_SUBSTEPS = 4;
      const btScalar SUBSTEP_LEN = 0.005; // 0.001; //1.0/60; // 0.01;
      m_clock.reset(); // 次回の測定の準備
#ifdef USE_PROF
      static int n = 0;
      n += m_dynamicsWorld->stepSimulation(dt, MAX_SUBSTEPS, SUBSTEP_LEN); // 実経過時間に合わせてシミュレーションを進める
      g_prof << n << " " << dt <<" " << (m_currentTime + dt)/n << "\n";
      // 理想的には(m_currentTIme + dt)/n → SUBSTEP_LEN
#else      
      m_dynamicsWorld->stepSimulation(dt, MAX_SUBSTEPS, SUBSTEP_LEN); // 実経過時間に合わせてシミュレーションを進める
#endif      
      m_currentTime += dt;
    }
    
    m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
  }

  displayCallback();
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  シミュレーション停止状態のとき（'i'キーが押されたとき）に
 *  シーンの描画のために呼ばれる関数（コールバック）
 */
void BPWorld::displayCallback(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  renderme();
  m_dynamicsWorld->debugDrawWorld();
  glutSwapBuffers();
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  キーボード入力処理のために呼ばれる関数（コールバック）
 */
void BPWorld::keyboardCallback(unsigned char key, int x, int y)
{
  this->DemoApplication::keyboardCallback(key, x, y);

  switch(key){
  case 'e':
    //btVector3 pc = getCameraPosition();
    //printf("cam pos: %f, %f, %f\n", pc.x(), pc.y(), pc.z());
    printf("cam. dist.: %f\n", getCameraDistance());
    printf("cam. azi.: %f\n", m_azi);
    printf("cam. ele.: %f\n", m_ele);
    printf("cam. up:   %f, %f, %f\n", m_cameraUp[0], m_cameraUp[1], m_cameraUp[2]);
    printf("cam. target pos.: %f, %f, %f\n",
           m_cameraTargetPosition[0],
           m_cameraTargetPosition[1],
           m_cameraTargetPosition[2]);
    break;
  }
  
  if(m_checkKey != NULL){
    m_checkKey(key, x, y);
  }
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  リセット時（スペースキーが押されたときなど）の処理
 */
void BPWorld::clientResetScene()
{
  this->DemoApplication::clientResetScene();
  m_currentTime = 0;
    
  m_autocam						=	false;
  m_raycast						=	false;
  m_cutting						=	false;
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  ジョイントの可動範囲の表示を有効にする
 */
void BPWorld::enableConstraintDisplay(bool onOff)
{
  const int CONSTRAINT_FLAGS =
    btIDebugDraw::DBG_DrawContactPoints |  
    btIDebugDraw::DBG_DrawConstraints | 
    btIDebugDraw::DBG_DrawConstraintLimits | // ジョイントの上限・下限の表示
    0;
  
  if(onOff){
    setDebugMode(getDebugMode() | CONSTRAINT_FLAGS);
  }else{
    setDebugMode(getDebugMode() & ~CONSTRAINT_FLAGS);
  }
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  柔軟物体をワールドに追加する
 */
void BPWorld::addSoftBody(btSoftBody* sb)
{
  ((btSoftRigidDynamicsWorld*)m_dynamicsWorld)->addSoftBody(sb);
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  デモの構成上，このメンバ関数が各物体を描画する際に呼び出される．
 *
 *  デモのGL_ShapeDrawerでは描けない物体（平面など）を描く機能を追加する
 *
 *  @param transMatrix 描画対象物体の位置・姿勢を表す行列
 *  @param shape 描画対象物体の形状
 *  @param color 色を表すRGB値
 *  @param debugMode デバッグモードを表すフラグ
 *  @param worldBoundsMin 描画対象領域を表す直方体(axis-aligned bounding box)の最小値側頂点に対応する点
 *  @param worldBoundsMax 描画対象領域を表す直方体(axis-aligned bounding box)の最大値側頂点に対応する点
 */
void BPWorld::NewShapeDrawer::drawOpenGL(btScalar* transMatrix,
                                         const btCollisionShape* shape,
                                         const btVector3& color, int debugMode,
                                         const btVector3& worldBoundsMin,
                                         const btVector3& worldBoundsMax)
{
    // 既存の描画関数で描ける物体は，既存の描画関数で描く
  this->GL_ShapeDrawer::drawOpenGL(transMatrix, shape, color, debugMode,
                                   worldBoundsMin, worldBoundsMax);

  if(m_textureenabled){ // テクスチャマッピングを行う場合
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,m_texturehandle);
  }else{
    glDisable(GL_TEXTURE_2D);
  }

  // 平面（を表す正方形）を描く
  if(shape->getShapeType() == STATIC_PLANE_PROXYTYPE){ 
    const btStaticPlaneShape *planeShape = static_cast<const btStaticPlaneShape*>(shape);
      
    // 平面の方程式は (n,r) = d. ここで r は3次元位置ベクトル変数．(n,r)は内積
    btVector3 n = planeShape->getPlaneNormal();   // 平面の法線ベクトル
    btScalar  d = planeShape->getPlaneConstant(); // 原点から平面に下ろした垂線の長さ
    const btScalar L = 600;//40.0; // 平面を表す正方形一辺の長さの半分
    btVector3 p0 = n * d; // 原点から平面に下ろした垂線の足を指す位置ベクトル
    btVector3 v0, v1; // nに垂直かつ，互いに直交するベクトル
    btPlaneSpace1(n, v0, v1); // v0, v1を求める．btPlaneSpace1() はbtVector3.hで定義されている．

    // 平面を表す矩形の四隅それぞれの位置p1, p2, p3, p4を計算する
    btVector3 p1 =  p0 + ( v0 + v1)*L;
    btVector3 p2 =  p0 + (-v0 + v1)*L;
    btVector3 p3 =  p0 + (-v0 - v1)*L;
    btVector3 p4 =  p0 + ( v0 - v1)*L;

    glBegin(GL_QUADS);
    {
      glNormal3f(n[0], n[1], n[2]); // この平面について照光処理を行うため
      
      glVertex3f(p1[0], p1[1], p1[2]);  
      glVertex3f(p2[0], p2[1], p2[2]);
      glVertex3f(p3[0], p3[1], p3[2]);
      glVertex3f(p4[0], p4[1], p4[2]);
    }
    glEnd();
  }

  if(m_textureenabled){
    glDisable(GL_TEXTURE_2D);
  }
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  カメラの位置，姿勢を設定する
 *
 *  @param ptx 注視点位置 [ptx, pty, ptz] の x成分
 *  @param pty 注視点位置 [ptx, pty, ptz] の y成分
 *  @param ptz 注視点位置 [ptx, pty, ptz] の z成分
 *  @param dist 注視点からカメラ視点までの距離 
 *  @param azi  方位角（azimuth） [deg]  ワールド座標系Z軸と，視線方向ベクトルのXZ平面への射影がなす角度
 *  @param ele  仰角（elevation） [deg]  XZ平面と視線方向ベクトルがなす角度
 *  @param upx 天井方向ベクトル [upx, upy, upz] の x成分（cf. GLのgluLookAt()）
 *  @param upy 天井方向ベクトル [upx, upy, upz] の y成分（cf. GLのgluLookAt()）
 *  @param upz 天井方向ベクトル [upx, upy, upz] の z成分（cf. GLのgluLookAt()）
 */
void BPWorld::setCamera(float ptx, float pty, float ptz,
                        float dist, float azi, float ele,
                        float upx, float upy, float upz)
{
  // これらのカメラ関連のメンバはDemoApplication.hで定義されている．
  m_cameraTargetPosition = btVector3(ptx, pty, ptz);
  setCameraDistance(dist);
  m_azi = azi;
  m_ele = ele;
  setCameraUp(btVector3(upx, upy, upz));
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  
 *  カメラの状態（投影行列およびモデル・ビュー行列）を更新する．
 *  BulletPhysicsライブラリ ver. 2.75の実装だとカメラの動きが
 *    |m_ele| = +90 deg付近で不連続（おそらく誤り）のため実装しなおす．
 */
void BPWorld::updateCamera()
{
  btScalar rele = btRadians(m_ele); // [rad]
  btScalar razi = btRadians(m_azi); // [rad] 
  btQuaternion rot(m_cameraUp, razi);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();


  btVector3 eyePos(0, 0, -m_cameraDistance);

  btVector3 forward(eyePos[0], eyePos[1], eyePos[2]);
  if(forward.length2() < SIMD_EPSILON){
    forward.setValue(1.f,0.f,0.f);
  }
  btVector3 right = m_cameraUp.cross(forward);
  btQuaternion roll(right, -rele);

  eyePos = quatRotate(rot * roll, eyePos);

  m_cameraPosition = eyePos + m_cameraTargetPosition;
    
  if(m_glutScreenWidth == 0 && m_glutScreenHeight == 0){
    return;
  }
  btScalar aspect;
  btVector3 extents;

  if(m_glutScreenWidth > m_glutScreenHeight){
    aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
    extents.setValue(aspect * 1.0f, 1.0f,0);
  }else{
    aspect = m_glutScreenHeight / (btScalar)m_glutScreenWidth;
    extents.setValue(1.0f, aspect*1.f,0);
  }

	
  if(m_ortho){
    // reset matrix
    glLoadIdentity();
		
		
    extents *= m_cameraDistance;
    btVector3 lower = m_cameraTargetPosition - extents;
    btVector3 upper = m_cameraTargetPosition + extents;
    //gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
    glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
		
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glTranslatef(100,210,0);
  }else{
    if(m_glutScreenWidth > m_glutScreenHeight){
      glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
      //glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
    }else {
      glFrustum (-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);
      //glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
    }

    btVector3 up;
    if(SIMD_HALF_PI <= fabs(rele)  &&  fabs(rele) <= (3.0/2.0)*SIMD_PI){
      up = - m_cameraUp;
    }else{
      up = m_cameraUp;
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 
              m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
              up[0], up[1], up[2]);
  }
  
}

////////////////////////////////////////////////////////////////////////////////
/**
 *  マウスドラッグ時の処理
 *  BulletPhysicsライブラリ ver. 2.75の実装だとカメラの動きが
 *    |m_azi| = +180 deg付近で不連続（おそらく誤り）のため実装しなおす．
 */
void BPWorld::mouseMotionFunc(int x, int y)
{
  extern float gOldPickingDist; // 実体の定義はDemoApplication.cppにあり
  
  // ピッキング時の処理
  if(m_pickConstraint){
    //move the constraint pivot
    btPoint2PointConstraint* p2p = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
    if(p2p){
      //keep it at the same picking distance
      btVector3 newRayTo = getRayTo(x,y);
      btVector3 rayFrom;
      btVector3 oldPivotInB = p2p->getPivotInB();
      btVector3 newPivotB;
      if (m_ortho){
        newPivotB = oldPivotInB;
        newPivotB.setX(newRayTo.getX());
        newPivotB.setY(newRayTo.getY());
      }else{
        rayFrom = m_cameraPosition;
        btVector3 dir = newRayTo-rayFrom;
        dir.normalize();
        dir *= gOldPickingDist;

        newPivotB = rayFrom + dir;
      }
      p2p->setPivotB(newPivotB);
    }

  }

  float dx, dy;
  dx = x - m_mouseOldX;
  dy = y - m_mouseOldY;


  // カメラの移動
  // （ALT (or CTRL) を押したままドラッグしているときの処理）
  if(m_modifierKeys & BT_ACTIVE_ALT){
    if(m_mouseButtons & 2){
      btVector3 hor  = getRayTo(0,0) - getRayTo(1,0);
      btVector3 vert = getRayTo(0,0) - getRayTo(0,1);
      btScalar multiplierX = 0.01;
      btScalar multiplierY = 0.01;
      if(m_ortho){
        multiplierX = 1;
        multiplierY = 1;
      }

      m_cameraTargetPosition += hor* dx * multiplierX;
      m_cameraTargetPosition += vert* dy * multiplierY;
    }

    if(m_mouseButtons & 1){
      m_azi += dx * 0.2;
      m_azi = fmodf(m_azi, 360.f);
      m_ele += dy * 0.2;
      m_ele = fmodf(m_ele, 360.f);
    }else if(m_mouseButtons & 4){
      m_cameraDistance -= dy * 0.2f;
      if(m_cameraDistance < SIMD_EPSILON){
        m_cameraDistance = SIMD_EPSILON;
      }
    } 
  }


  m_mouseOldX = x;
  m_mouseOldY = y;
  updateCamera();
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  （接触ジョイントを含む）ジョイントについてのERPのデフォルト値を指定する
 *
 *  @param erp Error Reduction Parameter (ERP)
 */
void BPWorld::setGlobalERP(btScalar erp)
{
  if(m_dynamicsWorld != NULL){
    btContactSolverInfo& slvInfo = m_dynamicsWorld->getSolverInfo();
    slvInfo.m_erp = erp;
  }
}

void	BPWorld::renderme()
{
    btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
    
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    m_dynamicsWorld->debugDrawWorld();
    
    //int debugMode = m_dynamicsWorld->getDebugDrawer()? m_dynamicsWorld->getDebugDrawer()->getDebugMode() : -1;
    
    btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
    //btIDebugDraw*	sdraw = softWorld ->getDebugDrawer();
    
    
    for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
    {
        btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
        if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
        {
            btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
            btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
        }
    }
    
    /* Bodies		*/
    btVector3	ps(0,0,0);
    int			nps=0;
    
    btSoftBodyArray&	sbs=softWorld->getSoftBodyArray();
    for(int ib=0;ib<sbs.size();++ib)
    {
        btSoftBody*	psb=sbs[ib];
        nps+=psb->m_nodes.size();
        for(int i=0;i<psb->m_nodes.size();++i)
        {
            ps+=psb->m_nodes[i].m_x;
        }
    }
    ps/=nps;
    if(m_autocam)
        m_cameraTargetPosition+=(ps-m_cameraTargetPosition)*0.05;
    /* Anm			*/
    if(!isIdle())
        m_animtime=m_clock.getTimeMilliseconds()/1000.f;
    /* Ray cast		*/
    if(m_raycast)
    {
        /* Prepare rays	*/
        const int		res=64;
        const btScalar	fres=res-1;
        const btScalar	size=8;
        const btScalar	dist=10;
        btTransform		trs;
        trs.setOrigin(ps);
        btScalar rayLength = 1000.f;
        
        const btScalar	angle=m_animtime*0.2;
        trs.setRotation(btQuaternion(angle,SIMD_PI/4,0));
        btVector3	dir=trs.getBasis()*btVector3(0,-1,0);
        trs.setOrigin(ps-dir*dist);
        btAlignedObjectArray<btVector3>	origins;
        btAlignedObjectArray<btScalar>	fractions;
        origins.resize(res*res);
        fractions.resize(res*res,1.f);
        for(int y=0;y<res;++y)
        {
            for(int x=0;x<res;++x)
            {
                const int	idx=y*res+x;
                origins[idx]=trs*btVector3(-size+size*2*x/fres,dist,-size+size*2*y/fres);
            }
        }
        /* Cast rays	*/
        {
            m_clock.reset();
            if (sbs.size())
            {
                btVector3*		org=&origins[0];
                btScalar*				fraction=&fractions[0];
                btSoftBody**			psbs=&sbs[0];
                btSoftBody::sRayCast	results;
                for(int i=0,ni=origins.size(),nb=sbs.size();i<ni;++i)
                {
                    for(int ib=0;ib<nb;++ib)
                    {
                        btVector3 rayFrom = *org;
                        btVector3 rayTo = rayFrom+dir*rayLength;
                        if(psbs[ib]->rayTest(rayFrom,rayTo,results))
                        {
                            *fraction=results.fraction;
                        }
                    }
                    ++org;++fraction;
                }
                long	ms=btMax<long>(m_clock.getTimeMilliseconds(),1);
                long	rayperseconds=(1000*(origins.size()*sbs.size()))/ms;
                printf("%d ms (%d rays/s)\r\n",int(ms),int(rayperseconds));
            }
        }
        /* Draw rays	*/
        const btVector3	c[]={	origins[0],
            origins[res-1],
            origins[res*(res-1)],
            origins[res*(res-1)+res-1]};
        idraw->drawLine(c[0],c[1],btVector3(0,0,0));
        idraw->drawLine(c[1],c[3],btVector3(0,0,0));
        idraw->drawLine(c[3],c[2],btVector3(0,0,0));
        idraw->drawLine(c[2],c[0],btVector3(0,0,0));
        for(int i=0,ni=origins.size();i<ni;++i)
        {
            const btScalar		fraction=fractions[i];
            const btVector3&	org=origins[i];
            if(fraction<1.f)
            {
                idraw->drawLine(org,org+dir*rayLength*fraction,btVector3(1,0,0));
            }
            else
            {
                idraw->drawLine(org,org-dir*rayLength*0.1,btVector3(0,0,0));
            }
        }
#undef RES
    }
    /* Water level	*/
    static const btVector3	axis[]={btVector3(1,0,0),
        btVector3(0,1,0),
        btVector3(0,0,1)};
    if(m_softBodyWorldInfo.water_density>0)
    {
        const btVector3	c=	btVector3((btScalar)0.25,(btScalar)0.25,1);
        const btScalar	a=	(btScalar)0.5;
        const btVector3	n=	m_softBodyWorldInfo.water_normal;
        const btVector3	o=	-n*m_softBodyWorldInfo.water_offset;
        const btVector3	x=	btCross(n,axis[n.minAxis()]).normalized();
        const btVector3	y=	btCross(x,n).normalized();
        const btScalar	s=	25;
        idraw->drawTriangle(o-x*s-y*s,o+x*s-y*s,o+x*s+y*s,c,a);
        idraw->drawTriangle(o-x*s-y*s,o+x*s+y*s,o-x*s+y*s,c,a);
    }
    //
    
    int lineWidth=280;
    int xStart = m_glutScreenWidth - lineWidth;
    int yStart = 20;
    
    if((getDebugMode() & btIDebugDraw::DBG_NoHelpText)==0)
    {
        setOrthographicProjection();
        glDisable(GL_LIGHTING);
        glColor3f(0, 0, 0);
        char buf[124];
        
        glRasterPos3f(xStart, yStart, 0);
//        if (sDemoMode)
//        {
//            sprintf(buf,"d to toggle demo mode (on)");
//        } else
//        {
//            sprintf(buf,"d to toggle demo mode (off)");
//        }
//        GLDebugDrawString(xStart,20,buf);
        glRasterPos3f(xStart, yStart, 0);
//        sprintf(buf,"] for next demo (%d)",current_demo);
        yStart+=20;
//        GLDebugDrawString(xStart,yStart,buf);
        glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"c to visualize clusters");
        yStart+=20;
//        GLDebugDrawString(xStart,yStart,buf);
        glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"; to toggle camera mode");
        yStart+=20;
//        GLDebugDrawString(xStart,yStart,buf);
        glRasterPos3f(xStart, yStart, 0);
        sprintf(buf,"n,m,l,k for power and steering");
        yStart+=20;
//        GLDebugDrawString(xStart,yStart,buf);
        
        
        resetPerspectiveProjection();
        glEnable(GL_LIGHTING);
    }
    
    DemoApplication::renderme();
    
}