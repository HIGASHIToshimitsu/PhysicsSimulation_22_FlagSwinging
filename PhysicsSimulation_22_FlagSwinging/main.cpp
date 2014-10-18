////////////////////////////////////////////////////////////////////////////////
/**
 *  旗のシミュレーション
 *
 */

#include "BPWorld.h"
#include <iostream>

btRigidBody *g_poleBody; //* 棒の力学データ
btSoftBody *g_flagBody;  //* 旗の布部分の力学データ
btScalar g_refAng = 0;   //* 目標角度
btHingeConstraint *g_joint; //* 棒を空間に対して固定するジョイント
bool g_autoModeRequested = false; //* 旗を自動で振るか否かを表すフラグ


////////////////////////////////////////////////////////////////////////////////
/**
 *  シミュレーションのステップごとに呼ばれる関数（コールバック）
 *  物体の状態をモニタリングするために使う．
 *
 *  @param currentTime 現在時刻
 */
void doMonitoring(btScalar currentTime){
    const btScalar MAX_MOTOR_TORQUE = 400; // モータのトルクの最大値
    const btScalar K = 2; // 比例ゲイン
    
    btScalar ang = g_joint->getHingeAngle();
    btScalar ref;
    
    if(g_autoModeRequested){
        const btScalar A = btRadians(45); // 振幅 [deg]
        const btScalar w = 2*M_PI*0.25; // 角周波数 [rad/s]
        ref = A*btSin(w*currentTime);   // 時間経過とともに正弦波状に変化する角度目標値
    }else{
        ref = btRadians(g_refAng);
    }
    
    // 比例制御
    g_joint->enableAngularMotor(true, K*(ref - ang), MAX_MOTOR_TORQUE);
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  キー入力を処理する
 */
void checkKey(int key, int x, int y){
    using namespace std;
    const btScalar DELTA = 1; // [deg]
    const btScalar LL_ANGLE = -90; // 角度下限 [deg]
    const btScalar UL_ANGLE =  90; // 角度上限 [deg]
    
    switch(key){
        case 'j':
            g_refAng += DELTA;
            break;
            
        case 'k':
            g_refAng -= DELTA;
            break;
            
        case 'A':
            g_autoModeRequested = !g_autoModeRequested;
            cout << ((g_autoModeRequested) ? "Automatic" : "Manual") << " mode selected" << endl;
            break;
    }
    
    if(g_refAng > UL_ANGLE){
        g_refAng = UL_ANGLE;
    }
    if(g_refAng < LL_ANGLE){
        g_refAng = LL_ANGLE;
    }
    if(!g_autoModeRequested){
        cout << "ref. angle = " << g_refAng << " deg" << endl;
    }
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  物体を作成して，ワールドに配置する
 */
void setUpObjects(BPWorld& bpWorld){
    const btScalar R = 0.2; // 棒（円柱）の半径
    const btScalar L = 8;   // 棒（円柱）の長さ
    btCylinderShape *poleShape = new btCylinderShape(btVector3(R, 0.5*L, R)); // 棒の形状データ
    const btScalar H = 3;   // 旗の布部分の高さ
    const btScalar W = 4;   // 旗の布部分の幅
    const int NUM_NODES_X = 15; // 布の分割節点個数（横方向）
    const int NUM_NODES_Y = 9;  // 布の分割節点個数（縦方向）
    const bool GEN_DIAGS = true; // 布の構成要素に対角方向のリンクを自動追加するか否か
    const btScalar OFFSET = 1;
    
    btTransform transform; // 座標変換データ
    transform.setIdentity();
    
    // 棒の作成
    transform.setOrigin(btVector3(0, 0.5*L + OFFSET, 0));
    g_poleBody = bpWorld.createRigidBody(0.1,        // 質量
                                         transform,  // 初期位置，姿勢
                                         poleShape); // 形状データ
    
    btVector3 pivot = btVector3(0, -0.5*L, 0); // ピボット位置（棒のローカル座標）
    btVector3 axis  = btVector3(0, 0, 1); // 回転軸の方向（棒のローカル座標）
    
    // 現在位置・姿勢を基準にして棒を空間に固定するジョイントを作る
    //  btHingeConstraintのコンストラクタに物体を一つだけ指定する場合，
    //  物体を空間（あるいは空間に固定された不可視物体）に対して
    //  ヒンジ・ジョイントで拘束することになる
    g_joint = new btHingeConstraint(*g_poleBody, pivot, axis);
    bpWorld.addConstraint(g_joint);
    
    
    // 旗の布部分を作成する
    // c01      c11
    //   ┌──┐
    //   ｜    ｜
    //   └──┘
    // c00      c10
    g_flagBody = btSoftBodyHelpers::CreatePatch(bpWorld.getSoftBodyWorldInfo(),
                                                btVector3(-R,   L-H + OFFSET, 0), // c00 (0x01)
                                                btVector3(-R-W, L-H + OFFSET, 0), // c10 (0x02)
                                                btVector3(-R,   L   + OFFSET, 0), // c01 (0x04)
                                                btVector3(-R-W, L   + OFFSET, 0), // c11 (0x08)
                                                NUM_NODES_X,
                                                NUM_NODES_Y,
                                                0, // 0x01 | 0x04, // 固定点を表すフラグ(1:c00, 4:c01),
                                                GEN_DIAGS);
    // 質量の設定
    g_flagBody->setTotalMass(0.001,
                             true); // trueなら布を構成する各三角面の面積を基に質量の分布を定める
    // falseなら現在の質量の比分布を維持して全体の質量が第一引数になるようにする
    
    // 棒に対して布の端点を固定する．
    g_flagBody->appendAnchor(0, g_poleBody); // 第1引数は固定する分割節点の番号
    g_flagBody->appendAnchor((NUM_NODES_Y-1)*NUM_NODES_X, g_poleBody);
    
    // 各パラメータのデフォルト値についてはbtSoftBody.cppのコンストラクタの記述を参照
    g_flagBody->m_cfg.kVCF = 1.0; // 速度修正パラメータ (VCF: velocity correction factor) [Baumgarte, 1972]?
    g_flagBody->m_cfg.kDP  = 0.0; // 速度減衰係数 (DP:damping)．値の範囲 [0, 1]．
    g_flagBody->m_cfg.kDG  = 0.0; // 抗力係数 (DG: drag)．値の範囲 [0, +∞]．（cf. 空気力学）
    g_flagBody->m_cfg.kLF  = 0.0; // 揚力係数 (LF: lift)．値の範囲 [0, +∞]．（cf. 空気力学）
    g_flagBody->m_cfg.kPR  = 0.0; // 圧力係数 (PR: pressure)．値の範囲 [-∞, +∞]（cf. 空気力学）
    g_flagBody->m_cfg.kVC  = 0.0; // 体積維持係数 (VC: volume conservation) [0, +∞]．ソースのコメントの'conversation'(会話)は誤記のはず．
    g_flagBody->m_cfg.kCHR = 1.0; // 剛体接触の堅さ (CHR: rigid body contacts hardness)．範囲 [0, 1]．
    g_flagBody->m_cfg.kKHR = 0.1; // 動的接触の堅さ (KHR: Kinetic contacts hardness)．範囲 [0, 1]．
    g_flagBody->m_cfg.kSHR = 1.0; // 柔軟体接触の堅さ (SHR: soft contacts hardness)
    g_flagBody->m_cfg.kAHR = 1.0; // アンカーの堅さ (AHR: anchors hardness)．範囲 [0, 1]．1に近づくほど堅さ大
    g_flagBody->m_cfg.viterations = 1; // 速度ソルバ反復計算回数 (velocities solver iterations)
    g_flagBody->m_cfg.piterations = 4; // 位置ソルバ反復計算回数 (positions solver iterations)．
    g_flagBody->m_cfg.diterations = 0; // ドリフトソルバ反復計算回数 (drift solver iterations)．
    // --> 反復計算回数が多いほど拘束の精度が向上する．
    //     e.g. piterartionsが少ない場合，棒と布の接続が離れる（kAHRの大きさにも依存）
    
    // 文献:
    //  Baumgarte, "Stabilization of Constraints and Integrals of Motion in Dynamical Systems",
    //  Computer Methods in Applied Mechanics and Engineering, 1, 1-16, 1972.
    
    bpWorld.addSoftBody(g_flagBody);
    
    // カメラの位置・姿勢の設定
    bpWorld.setCamera(0, 5, 2, // 注視点の位置 [x, y, z]
                      9,  // 注視点からカメラ視点までの距離
                      0,  // 方位角（azimuth） [deg]
                      20, // 仰角（elevation） [deg]
                      0, 1, 0); // 天井方向ベクトル [x, y, z] （cf. OpenGLのgluLookAt()）
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  使用法を表示する
 */
void showUsage()
{
    using namespace std;
    cout << "j/k: increase/decrease angle\n"
    << "A: toggle automatic control mode\n"
    << endl;
}


////////////////////////////////////////////////////////////////////////////////
/**
 *  プログラムの開始点
 */
int main(){
    // 動力学シミュレーションのワールドの準備
    BPWorld bpWorld("Flag Swinging", // ウィンドウのタイトル
                    640, 480,        // ウィンドウの幅と高さ [pixels]
                    BPWorld::SOFT_BODY_INCLUDED,
                    doMonitoring,    // モニタリング用関数へのポインタ
                    checkKey);       // キーボード入力処理用関数へのポインタ
    
    bpWorld.setGlobalERP(0.2);
    
    setUpObjects(bpWorld); // 物体を作成して配置する
    
    showUsage();
    // bpWorld.toggleIdle(); // ウィンドウ表示の後，シミュレーションを一時停止状態にするため
    
    bpWorld.start(); // ウィンドウを表示して，シミュレーションを開始する
    
    return 0;
}

