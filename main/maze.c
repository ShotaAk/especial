
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "maze.h"
#include "observer.h"
#include "variables.h"
#include "parameters.h"
#include "controller.h"
#include "logger.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Maze";

/*
 * Wall Status Bits: [00] -> [isKnown isWall]
 * Wall Mask: Search[01] -> 壁情報のみを抽出、 Second(2走目)[11] -> 知ってる かつ 壁があるところ抽出
 *
 */

#define MASK_SEARCH 0x01 // 探索走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なしor未探索区間
#define MASK_SECOND 0x03 // 最短走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なし
#define MAZESIZE_X (32)
#define MAZESIZE_Y (32)
#define UNKNOWN 2 // 壁があるかないか判らない状態の場合の値
#define NOWALL  0 // 壁がないばあいの値
#define WALL    1 // 壁がある場合の値
#define VWALL   3 // 仮想壁の値(未使用)
#define CONV_SEN2WALL(w) ((w) ? WALL : NOWALL)

typedef enum
{
    north=0,
    east=1,
    south=2,
    west=3,
}t_direction;

typedef struct
{
    short x;
    short y;
    t_direction dir;
}t_position;

typedef enum
{
    LOCAL_FRONT=0, // 前
    LOCAL_RIGHT=1, // 右
    LOCAL_REAR=2,  // 後
    LOCAL_LEFT=3,  // 左
    LOCAL_UNKNOWN, // 方向不明
}ENUM_LOCAL_DIRECTION; // 自分から見た方向を示す列挙型
typedef unsigned int t_local_dir;

typedef enum
{
    PRIORITY_LOWEST = 0,
    PRIORITY_LOW,
    PRIORITY_MID,
    PRIORITY_HIGH,
    PRIORITY_HIGHEST,
}ENUM_PRIORITY;
typedef unsigned int t_priority;

const unsigned char INIT_STEPS = 255;
const unsigned char MIN_STEP = 0;
static t_steps StepMap[MAZESIZE_X][MAZESIZE_Y]; // 歩数マップ
static t_wall WallMap[MAZESIZE_X][MAZESIZE_Y];

void initialzeStepMap(int goalX, int goalY)
{
    // 迷路の歩数Mapを初期化する。
    // 全体をINIT_STEPSで初期化し、ゴール座標golaX, golaYはMIN_STEPで初期化する
    
    for(int x_i = 0; x_i < MAZESIZE_X; x_i++) // 迷路の大きさ分ループ(x座標)
    {
        for(int y_i = 0; y_i < MAZESIZE_Y; y_i++) // 迷路の大きさ分ループ(y座標)
        {
            StepMap[x_i][y_i] = INIT_STEPS; // すべてINIT_STEPSで埋める
        }
    }

    StepMap[goalX][goalY] = MIN_STEP; // ゴール座標の歩数をMIN_STEPに設定
}


void makeStepMap(int goalX, int goalY, int mask) //歩数マップを作成する
{
    // 座標goalX,goalYをゴールとした歩数Mapを作成する。
    // maskの値(MASK_SEARCH or MASK_SECOND)によって、
    // 探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる

    initialzeStepMap(goalX, goalY); // 歩数Mapを初期化する

    int change_flag;			//Map作成終了を見極めるためのフラグ
    do
    {
        change_flag = FALSE; // 変更がなかった場合にはループを抜ける
        for(int x_i = 0; x_i < MAZESIZE_X; x_i++) // 迷路の大きさ分ループ(x座標)
        {
            for(int y_i = 0; y_i < MAZESIZE_Y; y_i++) // 迷路の大きさ分ループ(y座標)
            {
                if(StepMap[x_i][y_i] == INIT_STEPS) // INIT_STEPSの場合は次へ
                {
                    continue;
                }

                // 北方向の更新
                if(y_i < MAZESIZE_Y-1) // 配列の範囲外アクセス防止
                {
                    if( (WallMap[x_i][y_i].north & mask) == NOWALL)
                    {
                        // 上の区画の歩数を更新
                        if(StepMap[x_i][y_i+1] == INIT_STEPS)
                        {
                            StepMap[x_i][y_i+1] = StepMap[x_i][y_i] + 1;
                            change_flag = TRUE;
                        }
                    }
                }

                // 東方向の更新
                if(x_i < MAZESIZE_X-1) // 配列の範囲外アクセス防止
                {
                    if( (WallMap[x_i][y_i].east & mask) == NOWALL)
                    {
                        // 右の区画の歩数を更新
                        if(StepMap[x_i+1][y_i] == INIT_STEPS)
                        {
                            StepMap[x_i+1][y_i] = StepMap[x_i][y_i] + 1;
                            change_flag = TRUE;
                        }
                    }
                }

                // 南方向の更新
                if(y_i > 0) // 配列の範囲外アクセス防止
                {
                    if( (WallMap[x_i][y_i].south & mask) == NOWALL)
                    {
                        if(StepMap[x_i][y_i-1] == INIT_STEPS)
                        {
                            StepMap[x_i][y_i-1] = StepMap[x_i][y_i] + 1;
                            change_flag = TRUE;
                        }
                    }
                }

                // 西方向の更新
                if(x_i > 0) // 配列の範囲外アクセス防止
                {
                    if( (WallMap[x_i][y_i].west & mask) == NOWALL)
                    {
                        if(StepMap[x_i-1][y_i] == INIT_STEPS)
                        {
                            StepMap[x_i-1][y_i] = StepMap[x_i][y_i] + 1;
                            change_flag = TRUE;
                        }
                    }
                }
            } // y_i loop
        } // x_i loop
    }while(change_flag == TRUE); // 全体を作り終わるまで待つ
}

int isUnknown(int x, int y)
{
    // 座標x,yが未探索区間か否かを調べる

    if((WallMap[x][y].north == UNKNOWN) 
            || (WallMap[x][y].east == UNKNOWN) 
            || (WallMap[x][y].south == UNKNOWN) 
            || (WallMap[x][y].west == UNKNOWN))
    {
        return TRUE; // 未探索
    }
    else
    {
        return FALSE; // 探索済
    }
}

t_priority getPriority(int x, int y, t_direction dir, const t_position *mypos)
{
    // 座標x,yと、向いている方角dirから優先度を算出する

    // 未探索が一番優先度が高い.
    // それに加え、自分の向きと、行きたい方向から、
    // 前-> 横-> 後の順で優先度を付加する。

    t_priority priority = PRIORITY_LOWEST;

    if(mypos->dir == dir) // 行きたい方向が現在の進行方向と同じ場合
    {
        priority = PRIORITY_MID;
    }
    else if( ((4+mypos->dir-dir)%4) == 2) // 行きたい方向が現在の進行方向と逆の場合
    {
        priority = PRIORITY_LOWEST;
    }
    else // それ以外(左右どちらか)の場合
    {
        priority = PRIORITY_LOW;
    }


    if(isUnknown(x,y) == TRUE)
    {
        priority += PRIORITY_HIGHEST; // 未探索の場合優先度をさらに付加
    }

    return priority; // 優先度を返す
}

t_local_dir getNextDirection(const int goalX, const int goalY, const int mask, t_direction *dir,
        const t_position *mypos)
{
    // ゴール座標に向かう場合、今どちらに行くべきかを判断する。
    // 探索、最短の切り替えのためのmaskを指定、dirは方角を示す
    
    // stepsが少ない移動方向を探す
    t_steps lowestSteps = INIT_STEPS;
    t_priority priority = PRIORITY_LOWEST;

    t_priority tmp_priority;

    makeStepMap(goalX, goalY, mask); // 歩数Map生成

    // TODO:ここは同じことを４回書いてるので、もっとシンプルにできる

    if( (WallMap[mypos->x][mypos->y].north & mask) == NOWALL) // 北に壁がなければ
    {
        // 1区画先の歩数と優先度を取得
        int nextX = mypos->x;
        int nextY = mypos->y+1;
        t_steps nextSteps = StepMap[nextX][nextY];
        tmp_priority = getPriority(nextX, nextY, north, mypos);

        if(nextSteps < lowestSteps) // 一番歩数が小さい方向を見つける
        {
            lowestSteps = nextSteps; // ひとまず北が歩数が小さい事にする
            *dir = north; // 方向を保存
            priority = tmp_priority; // 優先度を保存
        }
        else if(nextSteps == lowestSteps) // 歩数が同じ場合は優先度から判断する
        {
            if(priority < tmp_priority ) // 優先度を評価
            {
                *dir = north; // 方向を更新
                priority = tmp_priority; // 優先度を保存
            }
        }
    }

    if( (WallMap[mypos->x][mypos->y].east & mask) == NOWALL) // 東に壁がなければ
    {
        // 1区画先の歩数と優先度を取得
        int nextX = mypos->x+1;
        int nextY = mypos->y;
        t_steps nextSteps = StepMap[nextX][nextY];
        tmp_priority = getPriority(nextX, nextY, east, mypos);

        if(nextSteps < lowestSteps) // 一番歩数が小さい方向を見つける
        {
            lowestSteps = nextSteps; // ひとまず東が歩数が小さい事にする
            *dir = east; // 方向を保存
            priority = tmp_priority; // 優先度を保存
        }
        else if(nextSteps == lowestSteps) // 歩数が同じ場合は優先度から判断する
        {
            if(priority < tmp_priority) // 優先度を評価
            {
                *dir = east; // 方向を保存
                priority = tmp_priority; // 優先度を保存
            }
        }
    }

    if( (WallMap[mypos->x][mypos->y].south & mask) == NOWALL) // 南に壁がなければ
    {
        // 1区画先の歩数と優先度を取得
        int nextX = mypos->x;
        int nextY = mypos->y-1;
        t_steps nextSteps = StepMap[nextX][nextY];
        tmp_priority = getPriority(nextX, nextY, south, mypos);

        if(nextSteps < lowestSteps) // 一番歩数が小さい方向を見つける
        {
            lowestSteps = nextSteps; // ひとまず南が歩数が小さい事にする
            *dir = south; // 方向を保存
            priority = tmp_priority; // 優先度を保存
        }
        else if(nextSteps == lowestSteps) // 歩数が同じ場合は優先度から判断する
        {
            if(priority < tmp_priority) // 優先度を評価
            {
                *dir = south; // 方向を保存
                priority = tmp_priority; // 優先度を保存
            }
        }
    }

    if( (WallMap[mypos->x][mypos->y].west & mask) == NOWALL) // 西に壁がなければ
    {
        // 1区画先の歩数と優先度を取得
        int nextX = mypos->x-1;
        int nextY = mypos->y;
        t_steps nextSteps = StepMap[nextX][nextY];
        tmp_priority = getPriority(nextX, nextY, west, mypos);

        if(nextSteps < lowestSteps) // 一番歩数が小さい方向を見つける
        {
            lowestSteps = nextSteps; // ひとまず西が歩数が小さい事にする
            *dir = west; // 方向を保存
            priority = tmp_priority; // 優先度を保存
        }
        else if(nextSteps == lowestSteps) // 歩数が同じ場合は優先度から判断する
        {
            *dir = west; // 方向を保存
            priority = tmp_priority; // 優先度を保存
        }
    }

    // TODO:ここはわかりにくい
    return ( (int)( ( 4 + *dir - mypos->dir) % 4 ) ); // どっちに向かうべきかを返す。
}

void setWall(const t_position *mypos)
{
    // 自分のいる座標に壁情報を書き込む
    
    int n_write=NOWALL,s_write=NOWALL,e_write=NOWALL,w_write=NOWALL;

    int x = mypos->x;
    int y = mypos->y;

    // 自分の方向に応じて書き込むデータを生成
    switch(mypos->dir){
        case north:
            n_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]); // 前壁の有無を判断
            e_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]); // 右壁の有無を判断
            w_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]); // 左壁の有無を判断
            s_write = NOWALL; // 後ろは必ず壁がない
            break;

        case east:
            e_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]); // 前壁の有無を判断
            s_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]); // 右壁の有無を判断
            n_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]); // 左壁の有無を判断
            w_write = NOWALL; // 後ろは必ず壁がない
            break;

        case south:
            s_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]); // 前壁の有無を判断
            w_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]); // 右壁の有無を判断
            e_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]); // 左壁の有無を判断
            n_write = NOWALL; // 後ろは必ず壁がない
            break;

        case west:
            w_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]); // 前壁の有無を判断
            n_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]); // 右壁の有無を判断
            s_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]); // 左壁の有無を判断
            e_write = NOWALL; // 後ろは必ず壁がない
            break;
    }
    WallMap[x][y].north = n_write;
    WallMap[x][y].south = s_write;
    WallMap[x][y].east  = e_write;
    WallMap[x][y].west  = w_write;

    // 周りの区画の壁情報も更新する
    if(y < MAZESIZE_Y-1) // 配列の範囲外アクセス防止
    {
        WallMap[x][y+1].south = n_write;
    }
    if(x < MAZESIZE_X-1)
    {
        WallMap[x+1][y].west = e_write;
    }
    if(y > 0)
    {
        WallMap[x][y-1].north = s_write;
    }
    if(x > 0)
    {
        WallMap[x-1][y].east = w_write;
    }
}

void initMaze(void)
{
    // 迷路情報の初期化
    for(int x_i = 0; x_i < MAZESIZE_X; x_i++)
    {
        for(int y_i = 0; y_i < MAZESIZE_Y; y_i++)
        {
            WallMap[x_i][y_i].north = 
                WallMap[x_i][y_i].east = 
                WallMap[x_i][y_i].south = 
                WallMap[x_i][y_i].west = UNKNOWN; // 迷路の全体がわからない事を設定する
        }
    }

    // 一番外側の壁を追加する
    // 上下
    for(int x_i = 0; x_i < MAZESIZE_X; x_i++)
    {
        WallMap[x_i][0].south = WALL; // 四方の壁を追加する(南)
        WallMap[x_i][MAZESIZE_Y-1].north = WALL; // 四方の壁を追加する(北)
    }

    // 左右
    for(int y_i = 0; y_i < MAZESIZE_Y; y_i++)
    {
        WallMap[0][y_i].west = WALL; // 四方の壁を追加する(西)
        WallMap[MAZESIZE_X-1][y_i].east = WALL; // 四方の壁を追加する(東)
    }

    // スタート地点の右の壁を追加
    WallMap[0][0].east = WallMap[1][0].west = WALL; // スタート地点の右の壁を追加する
}

void searchAdachi(const int goalX, const int goalY, const int slalomEnable, 
        const int doInitKetsuate,
        t_position *mypos){
    //引数goalX,goalYに向かって足立法で迷路を探索する
    float endSpeed = pSEARCH_MAX_SPEED;

    t_direction glob_nextdir; // 次に向かう方向を記録する変数

    // 移動距離を初期化
    gObsMovingDistance = 0;

    gMotorState = MOTOR_ON;

    // スタート直後のけつあて調整
    if(doInitKetsuate){
        ketsuate(endSpeed);
    }

    switch(getNextDirection(goalX,goalY,MASK_SEARCH,&glob_nextdir,mypos)) // 次に行く方向を戻り値とする関数を呼ぶ
    {
        case LOCAL_FRONT:

            searchStraight(pHALF_CELL_DISTANCE, endSpeed);
            break;

        case LOCAL_RIGHT:
            turn(-M_PI_2, pSEARCH_TIMEOUT);
            searchStraight(pHALF_CELL_DISTANCE, endSpeed);
            break;

        case LOCAL_LEFT:
            turn(M_PI_2, pSEARCH_TIMEOUT);
            searchStraight(pHALF_CELL_DISTANCE, endSpeed);
            break;

        case LOCAL_REAR:
            turn(M_PI, pSEARCH_TIMEOUT);
            searchStraight(pHALF_CELL_DISTANCE, endSpeed);
            break;
    }

    mypos->dir = glob_nextdir; // 方向を更新

    // 向いた方向によって自分の座標を更新する
    switch(mypos->dir)
    {
        case north:
            mypos->y++; // 北を向いた時はY座標を増やす
            break;

        case east:
            mypos->x++; // 東を向いた時はX座標を増やす
            break;

        case south:
            mypos->y--; // 南を向いた時はY座標を減らす
            break;

        case west:
            mypos->x--; // 西を向いたときはX座標を減らす
            break;
    }


    int doHipAdjust = 0; // けつあて補正
    int toggleBlink = 0; // １区画ごとに点灯と点滅を切り替える
    while((mypos->x != goalX) || (mypos->y != goalY)){ // ゴールするまで繰り返す

        setWall(mypos); // 壁をセット

        // 次に行く方向を戻り値とする関数を呼ぶ
        switch(getNextDirection(goalX,goalY,MASK_SEARCH,&glob_nextdir,mypos)) 
        {
            case LOCAL_FRONT:
                gIndicatorValue = 0 + 3*toggleBlink; // デバッグ用のLED点灯
                searchStraight(pCELL_DISTANCE, endSpeed);
                break;

            case LOCAL_RIGHT:
                gIndicatorValue = 0 + 3*toggleBlink; // デバッグ用のLED点灯
                if(slalomEnable){
                    slalom(TRUE, endSpeed, pSEARCH_TIMEOUT);
                }else{
                    if(gObsIsWall[DIREC_LEFT] == 1){
                        // 左に壁があればけつあて
                        doHipAdjust = 1;
                    }
                    searchStraight(pHALF_CELL_DISTANCE, 0.0);
                    turn(-M_PI_2, pSEARCH_TIMEOUT);
                    if(doHipAdjust){
                        ketsuate(endSpeed);
                        doHipAdjust = 0;
                    }
                    searchStraight(pHALF_CELL_DISTANCE, endSpeed);
                }
                break;

            case LOCAL_LEFT:
                gIndicatorValue = 0 + 3*toggleBlink; // デバッグ用のLED点灯
                if(slalomEnable){
                    slalom(FALSE, endSpeed, pSEARCH_TIMEOUT);
                }else{
                    if(gObsIsWall[DIREC_RIGHT] == 1){
                        // 右に壁があればけつあて
                        doHipAdjust = 1;
                    }
                    searchStraight(pHALF_CELL_DISTANCE, 0.0);
                    turn(M_PI_2, pSEARCH_TIMEOUT);
                    if(doHipAdjust){
                        ketsuate(endSpeed);
                        doHipAdjust = 0;
                    }
                    searchStraight(pHALF_CELL_DISTANCE, endSpeed);
                }
                break;

            case LOCAL_REAR:
                gIndicatorValue = 6; // デバッグ用のLED点灯
                if(gObsIsWall[DIREC_FRONT] == 1){
                    // 前に壁があればけつあて
                    doHipAdjust = 1;
                }
                searchStraight(pHALF_CELL_DISTANCE, 0.0);
                turn(M_PI, pSEARCH_TIMEOUT);
                if(doHipAdjust){
                    ketsuate(endSpeed);
                    doHipAdjust = 0;
                }
                searchStraight(pHALF_CELL_DISTANCE, endSpeed);
                break;
        }

        mypos->dir = glob_nextdir; // 方向を更新

        // 向いた方向によって自分の座標を更新する
        switch(mypos->dir)
        {
            case north:
                mypos->y++; // 北を向いた時はY座標を増やす
                break;

            case east:
                mypos->x++; // 東を向いた時はX座標を増やす
                break;

            case south:
                mypos->y--; // 南を向いた時はY座標を減らす
                break;

            case west:
                mypos->x--; // 西を向いたときはX座標を減らす
                break;
        }

        // LEDの点灯と点滅を切り替える
        if(toggleBlink==0){
            toggleBlink=1;
        }else{
            toggleBlink=0;
        }
    }

    setWall(mypos); // 壁をセット
    searchStraight(pHALF_CELL_DISTANCE, 0.0);
    // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);	
    // 制御終了状態
    gMotorDuty[RIGHT] = 0;
    gMotorDuty[LEFT] = 0;

    gMotorState = MOTOR_OFF;
}

void fastRun(const int goalX, const int goalY, const int slalomEnable, 
        const int doInitKetsuate, t_position *mypos){

    //引数goalX,goalYに向かって最短走行する
    float endSpeed = pFAST_MAX_SPEED;

    t_direction glob_nextdir; // 次に向かう方向を記録する変数

    // 移動距離を初期化
    gObsMovingDistance = 0;

    gMotorState = MOTOR_ON;

    // スタート直後のけつあて調整
    if(doInitKetsuate){
        ketsuate(endSpeed);
    }

    int straightCount=0; // 直進区画を連続走行するためのカウンタ

    // 現在の向きから、次に行くべき方向を向く
    switch(getNextDirection(goalX,goalY,MASK_SECOND,&glob_nextdir,mypos)) // 次に行く方向を戻り値とする関数を呼ぶ
    {
        case LOCAL_FRONT:
            // すぐに前進はせず、あとでまとめて前進する
            straightCount++;
            break;

        case LOCAL_RIGHT:
            turn(-M_PI_2, pFAST_TIMEOUT);
            straightCount = 1;
            break;

        case LOCAL_LEFT:
            turn(M_PI_2, pFAST_TIMEOUT);
            straightCount = 1;
            break;

        case LOCAL_REAR:
            turn(M_PI, pFAST_TIMEOUT);
            straightCount = 1;
            break;
    }

    mypos->dir = glob_nextdir; // 方向を更新

    // 向いた方向によって自分の座標を更新する
    switch(mypos->dir)
    {
        case north:
            mypos->y++; // 北を向いた時はY座標を増やす
            break;

        case east:
            mypos->x++; // 東を向いた時はX座標を増やす
            break;

        case south:
            mypos->y--; // 南を向いた時はY座標を減らす
            break;

        case west:
            mypos->x--; // 西を向いたときはX座標を減らす
            break;
    }

    while((mypos->x != goalX) || (mypos->y != goalY)){ // ゴールするまで繰り返す

        // 次に行く方向を戻り値とする関数を呼ぶ
        switch(getNextDirection(goalX,goalY,MASK_SECOND,&glob_nextdir,mypos)) 
        {
            case LOCAL_FRONT:
                straightCount++;
                break;

            case LOCAL_RIGHT:
                if(slalomEnable){
                    slalom(TRUE, endSpeed, pFAST_TIMEOUT);
                }else{
                    fastStraight(pCELL_DISTANCE * straightCount, 0.0);
                    turn(-M_PI_2, pFAST_TIMEOUT);
                    straightCount = 1; // 直線走行距離をリセット
                }
                break;

            case LOCAL_LEFT:
                if(slalomEnable){
                    slalom(FALSE, endSpeed, pFAST_TIMEOUT);
                }else{
                    fastStraight(pCELL_DISTANCE * straightCount, 0.0);
                    turn(M_PI_2, pFAST_TIMEOUT);
                    straightCount = 1; // 直線走行距離をリセット
                }
                break;

            case LOCAL_REAR:
                fastStraight(pCELL_DISTANCE * straightCount, 0.0);
                turn(M_PI, pFAST_TIMEOUT);
                straightCount = 1; // 直線走行距離をリセット
                break;
        }

        mypos->dir = glob_nextdir; // 方向を更新

        // 向いた方向によって自分の座標を更新する
        switch(mypos->dir)
        {
            case north:
                mypos->y++; // 北を向いた時はY座標を増やす
                break;

            case east:
                mypos->x++; // 東を向いた時はX座標を増やす
                break;

            case south:
                mypos->y--; // 南を向いた時はY座標を減らす
                break;

            case west:
                mypos->x--; // 西を向いたときはX座標を減らす
                break;
        }
    }
    fastStraight(pCELL_DISTANCE * straightCount, 0.0);
}


void search(const int goalX, const int goalY, const int slalomEnable, 
        const int goHomeEnable){

    int doInitKetsuate = TRUE;
    t_position myPos;
    myPos.x = myPos.y = 0;
    myPos.dir = north;

    // LEDを点灯するからこの間に指を離してね
    gIndicatorValue = 6;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gIndicatorValue = 9;

    searchAdachi(goalX,goalY,slalomEnable,doInitKetsuate,&myPos);
    // loggingSaveWall(WallMap,MAZESIZE_X,MAZESIZE_Y);
    // loggingSaveWall(MAZESIZE_X,MAZESIZE_Y,WallMap);

    // ゴールしたらLEDを点灯
    gIndicatorValue = 6;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    // スタート地点に戻る
    if(goHomeEnable){
        doInitKetsuate = FALSE;
        searchAdachi(0,0,slalomEnable,doInitKetsuate,&myPos);
    }

    // LEDを点灯するからこの間にEspecialを持ち上げてね
    gIndicatorValue = 9;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    gIndicatorValue = 0;

    // ダイアルを初期化
    gObsDial = 0;
}


void run(const int goalX, const int goalY, const int slalomEnable, 
        const int goHomeEnable){

    int doInitKetsuate = TRUE;

    t_position myPos;
    myPos.x = myPos.y = 0;
    myPos.dir = north;

    // LEDを点灯するからこの間に指を離してね
    gIndicatorValue = 6;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gIndicatorValue = 9;

    fastRun(goalX, goalX, slalomEnable, doInitKetsuate, &myPos);

    // ゴールしたらLEDを点灯
    gIndicatorValue = 6;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    // スタート地点に戻る
    if(goHomeEnable){
        doInitKetsuate = FALSE;
        fastRun(0, 0, slalomEnable, doInitKetsuate, &myPos);
    }

    // LEDを点灯するからこの間にEspecialを持ち上げてね
    gIndicatorValue = 9;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    gIndicatorValue = 0;

    // ダイアルを初期化
    gObsDial = 0;
}


