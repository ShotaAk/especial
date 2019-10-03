
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "observer.h"
#include "variables.h"
#include "controller.h"


#define MASK_SEARCH		0x01		//探索走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なしor未探索区間
#define MASK_SECOND		0x03		//最短走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なし
#define MAZESIZE_X (32)
#define MAZESIZE_Y (32)
#define UNKNOWN	2				//壁があるかないか判らない状態の場合の値
#define NOWALL	0				//壁がないばあいの値
#define WALL	1				//壁がある場合の値
#define VWALL	3				//仮想壁の値(未使用)
#define CONV_SEN2WALL(w) ((w) ? WALL : NOWALL)

typedef struct
{
    unsigned char north:2;	//北の壁情報
    unsigned char east:2;	//東の壁情報
    unsigned char south:2;	//南の壁情報
    unsigned char west:2;	//西の壁情報
}t_wall;			//壁情報を格納する構造体(ビットフィールド)

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
	front=0,		//前
	right=1,		//右
	rear=2,			//後
	left=3,			//左
	unknown,		//方向不明
}t_local_dir;	//自分から見た方向を示す列挙型

static unsigned char map[MAZESIZE_X][MAZESIZE_Y];
static t_wall wall[MAZESIZE_X][MAZESIZE_Y];
static t_position mypos;

void init_map(int x, int y)
{
    //迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する
    for(int i = 0; i < MAZESIZE_X; i++)		//迷路の大きさ分ループ(x座標)
    {
        for(int j = 0; j < MAZESIZE_Y; j++)	//迷路の大きさ分ループ(y座標)
        {
            map[i][j] = 255;	//すべて255で埋める
        }
    }

    map[x][y] = 0;				//ゴール座標の歩数を０に設定
}


void make_map(int x, int y, int mask)	//歩数マップを作成する
{
    // 座標x,yをゴールとした歩数Mapを作成する。
    // maskの値(MASK_SEARCH or MASK_SECOND)によって、
    // 探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる
    int i,j;
    int change_flag;			//Map作成終了を見極めるためのフラグ

    init_map(x,y);				//Mapを初期化する

    do
    {
        change_flag = FALSE;				//変更がなかった場合にはループを抜ける
        for(i = 0; i < MAZESIZE_X; i++)			//迷路の大きさ分ループ(x座標)
        {
            for(j = 0; j < MAZESIZE_Y; j++)		//迷路の大きさ分ループ(y座標)
            {
                if(map[i][j] == 255)		//255の場合は次へ
                {
                    continue;
                }

                if(j < MAZESIZE_Y-1)					//範囲チェック
                {
                    if( (wall[i][j].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
                    {
                        if(map[i][j+1] == 255)			//まだ値が入っていなければ
                        {
                            map[i][j+1] = map[i][j] + 1;	//値を代入
                            change_flag = TRUE;		//値が更新されたことを示す
                        }
                    }
                }

                if(i < MAZESIZE_X-1)					//範囲チェック
                {
                    if( (wall[i][j].east & mask) == NOWALL)		//壁がなければ
                    {
                        if(map[i+1][j] == 255)			//値が入っていなければ
                        {
                            map[i+1][j] = map[i][j] + 1;	//値を代入
                            change_flag = TRUE;		//値が更新されたことを示す
                        }
                    }
                }

                if(j > 0)						//範囲チェック
                {
                    if( (wall[i][j].south & mask) == NOWALL)	//壁がなければ
                    {
                        if(map[i][j-1] == 255)			//値が入っていなければ
                        {
                            map[i][j-1] = map[i][j] + 1;	//値を代入
                            change_flag = TRUE;		//値が更新されたことを示す
                        }
                    }
                }

                if(i > 0)						//範囲チェック
                {
                    if( (wall[i][j].west & mask) == NOWALL)		//壁がなければ
                    {
                        if(map[i-1][j] == 255)			//値が入っていなければ
                        {
                            map[i-1][j] = map[i][j] + 1;	//値を代入	
                            change_flag = TRUE;		//値が更新されたことを示す
                        }

                    }

                }

            }

        }

    }while(change_flag == TRUE);	//全体を作り終わるまで待つ

}

int is_unknown(int x, int y)	//指定された区画が未探索か否かを判断する関数 未探索:TRUE　探索済:FALSE
{
	//座標x,yが未探索区間か否かを調べる
	
	if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
	{			//どこかの壁情報が不明のままであれば
		return TRUE;	//未探索
	}
	else
	{
		return FALSE;	//探索済
	}
}

int get_priority(int x, int y, t_direction dir)	//そのマスの情報から、優先度を算出する
{
    //座標x,yと、向いている方角dirから優先度を算出する

    //未探索が一番優先度が高い.(4)
    //それに加え、自分の向きと、行きたい方向から、
    //前(2)横(1)後(0)の優先度を付加する。

    int priority;	//優先度を記録する変数

    priority = 0;

    if(mypos.dir == dir)				//行きたい方向が現在の進行方向と同じ場合
    {
        priority = 2;
    }
    else if( ((4+mypos.dir-dir)%4) == 2)		//行きたい方向が現在の進行方向と逆の場合
    {
        priority = 0;
    }
    else						//それ以外(左右どちらか)の場合
    {
        priority = 1;
    }


    if(is_unknown(x,y) == TRUE)
    {
        priority += 4;				//未探索の場合優先度をさらに付加
    }

    return priority;				//優先度を返す

}

int get_nextdir(int x, int y, int mask, t_direction *dir)	
{
    //ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
    //探索、最短の切り替えのためのmaskを指定、dirは方角を示す
    int little,priority,tmp_priority;		//最小の値を探すために使用する変数


    make_map(x,y,mask);				//歩数Map生成
    little = 255;					//最小歩数を255歩(mapがunsigned char型なので)に設定	

    priority = 0;					//優先度の初期値は0

    //maskの意味はstatic_parameter.hを参照
    if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//北に壁がなければ
    {
        tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//優先度を算出
        if(map[mypos.x][mypos.y+1] < little)				//一番歩数が小さい方向を見つける
        {
            little = map[mypos.x][mypos.y+1];			//ひとまず北が歩数が小さい事にする
            *dir = north;						//方向を保存
            priority = tmp_priority;				//優先度を保存
        }
        else if(map[mypos.x][mypos.y+1] == little)			//歩数が同じ場合は優先度から判断する
        {
            if(priority < tmp_priority )				//優先度を評価
            {
                *dir = north;					//方向を更新
                priority = tmp_priority;			//優先度を保存
            }
        }
    }

    if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)			//東に壁がなければ
    {
        tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//優先度を算出
        if(map[mypos.x + 1][mypos.y] < little)				//一番歩数が小さい方向を見つける
        {
            little = map[mypos.x+1][mypos.y];			//ひとまず東が歩数が小さい事にする
            *dir = east;						//方向を保存
            priority = tmp_priority;				//優先度を保存
        }
        else if(map[mypos.x + 1][mypos.y] == little)			//歩数が同じ場合、優先度から判断
        {
            if(priority < tmp_priority)				//優先度を評価
            {
                *dir = east;					//方向を保存
                priority = tmp_priority;			//優先度を保存
            }
        }
    }

    if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//南に壁がなければ
    {
        tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//優先度を算出
        if(map[mypos.x][mypos.y - 1] < little)				//一番歩数が小さい方向を見つける
        {
            little = map[mypos.x][mypos.y-1];			//ひとまず南が歩数が小さい事にする
            *dir = south;						//方向を保存
            priority = tmp_priority;				//優先度を保存
        }
        else if(map[mypos.x][mypos.y - 1] == little)			//歩数が同じ場合、優先度で評価
        {
            if(priority < tmp_priority)				//優先度を評価
            {
                *dir = south;					//方向を保存
                priority = tmp_priority;			//優先度を保存
            }
        }
    }

    if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)			//西に壁がなければ
    {
        tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//優先度を算出
        if(map[mypos.x-1][mypos.y] < little)				//一番歩数が小さい方向を見つける
        {
            little = map[mypos.x-1][mypos.y];			//西が歩数が小さい
            *dir = west;						//方向を保存
            priority = tmp_priority;				//優先度を保存
        }
        else if(map[mypos.x - 1][mypos.y] == little)			//歩数が同じ場合、優先度で評価
        {
            *dir = west;						//方向を保存
            priority = tmp_priority;				//優先度を保存
        }
    }


    return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );			//どっちに向かうべきかを返す。
    //演算の意味はmytyedef.h内のenum宣言から。

}

void set_wall(int x, int y)	//壁情報を記録
{
    //引数の座標x,yに壁情報を書き込む
    int n_write=NOWALL,s_write=NOWALL,e_write=NOWALL,w_write=NOWALL;


    //自分の方向に応じて書き込むデータを生成
    //CONV_SEN2WALL()はmacro.hを参照
    switch(mypos.dir){
        case north:	//北を向いている時

            // n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
            // e_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
            // w_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
            // s_write = NOWALL;						//後ろは必ず壁がない
            n_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]);	//前壁の有無を判断
            e_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]);				//右壁の有無を判断
            w_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]);				//左壁の有無を判断
            s_write = NOWALL;						//後ろは必ず壁がない

            break;

        case east:	//東を向いているとき

            // e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
            // s_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
            // n_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
            // w_write = NOWALL;						//後ろは必ず壁がない
            e_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]);	//前壁の有無を判断
            s_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]);				//右壁の有無を判断
            n_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]);				//左壁の有無を判断
            w_write = NOWALL;						//後ろは必ず壁がない

            break;

        case south:	//南を向いているとき

            // s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
            // w_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
            // e_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
            // n_write = NOWALL;						//後ろは必ず壁がない
            s_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]);	//前壁の有無を判断
            w_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]);				//右壁の有無を判断
            e_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]);				//左壁の有無を判断
            n_write = NOWALL;						//後ろは必ず壁がない

            break;

        case west:	//西を向いているとき

            // w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
            // n_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
            // s_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
            // e_write = NOWALL;						//後ろは必ず壁がない
            w_write = CONV_SEN2WALL(gObsIsWall[DIREC_FRONT]);	//前壁の有無を判断
            n_write = CONV_SEN2WALL(gObsIsWall[DIREC_RIGHT]);				//右壁の有無を判断
            s_write = CONV_SEN2WALL(gObsIsWall[DIREC_LEFT]);				//左壁の有無を判断
            e_write = NOWALL;						//後ろは必ず壁がない

            break;

    }

    wall[x][y].north = n_write;	//実際に壁情報を書き込み
    wall[x][y].south = s_write;	//実際に壁情報を書き込み
    wall[x][y].east  = e_write;	//実際に壁情報を書き込み
    wall[x][y].west  = w_write;	//実際に壁情報を書き込み

    if(y < MAZESIZE_Y-1)	//範囲チェック
    {
        wall[x][y+1].south = n_write;	//反対側から見た壁を書き込み
    }

    if(x < MAZESIZE_X-1)	//範囲チェック
    {
        wall[x+1][y].west = e_write;	//反対側から見た壁を書き込み
    }

    if(y > 0)	//範囲チェック
    {
        wall[x][y-1].north = s_write;	//反対側から見た壁を書き込み
    }

    if(x > 0)	//範囲チェック
    {
        wall[x-1][y].east = w_write;	//反対側から見た壁を書き込み
    }

}

void init_maze(void)	//迷路情報の初期化
{
	int i,j;
	
	for(i = 0; i < MAZESIZE_X; i++)
	{
		for(j = 0; j < MAZESIZE_Y; j++)
		{
			wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;	//迷路の全体がわからない事を設定する
		}
	}
	
	for(i = 0; i < MAZESIZE_X; i++)
	{
		wall[i][0].south = WALL;		//四方の壁を追加する(南)
		wall[i][MAZESIZE_Y-1].north = WALL;	//四方の壁を追加する(北)
	}
	
	for(j = 0; j < MAZESIZE_Y; j++)
	{
		wall[0][j].west = WALL;			//四方の壁を追加する(西)
		wall[MAZESIZE_X-1][j].east = WALL;	//四方の壁を追加する(東)
	}
	
	wall[0][0].east = wall[1][0].west = WALL;	//スタート地点の右の壁を追加する
	
}

void searchAdachi(const int goalX, const int goalY){
    static const char *TAG="ADACHI";
    //引数goalX,goalYに向かって足立法で迷路を探索する
    const float TIME_OUT = 4.0; // sec
    const float MAX_SPEED = 0.3; // m/s
    const float ACCEL = 1.0; // m/ss
    const float HALF_DISTANCE = 0.045;
    const float DISTANCE = 0.090;
    const float KETSU_DISTANCE = 0.003;
    const float KETSU_TIME_OUT = 0.5; // sec

    int result = TRUE;
    t_direction glob_nextdir;					//次に向かう方向を記録する変数

    // パラメータ初期化
    init_maze();
    mypos.x = mypos.y = 0;
    mypos.dir = north;

    gIndicatorValue = 6;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gIndicatorValue = 9;


    // ジャイロのバイアスリセット
    gGyroBiasResetRequest = 1;
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    gMotorState = MOTOR_ON;

    switch(get_nextdir(goalX,goalY,MASK_SEARCH,&glob_nextdir))		//次に行く方向を戻り値とする関数を呼ぶ
    {
        case front:

            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
            break;

        case right:
            result = turn(-M_PI_2, TIME_OUT);
            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
            break;

        case left:
            result = turn(M_PI_2, TIME_OUT);
            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
            break;

        case rear:
            result = turn(M_PI, TIME_OUT);
            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
            break;
    }
    // accel=SEARCH_ACCEL;				//加速度を設定
    // con_wall.enable = TRUE;					//壁制御を有効にする
    //MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
    // len_mouse = 0;					//進んだ距離カウント用変数をリセット
    // MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1;		//カウントスタート

    mypos.dir = glob_nextdir;				//方向を更新


    //向いた方向によって自分の座標を更新する
    switch(mypos.dir)
    {
        case north:
            mypos.y++;	//北を向いた時はY座標を増やす
            break;

        case east:
            mypos.x++;	//東を向いた時はX座標を増やす
            break;

        case south:
            mypos.y--;	//南を向いた時はY座標を減らす
            break;

        case west:
            mypos.x--;	//西を向いたときはX座標を減らす
            break;

    }


    int doHipAdjust = 0; // けつあて補正
    while((mypos.x != goalX) || (mypos.y != goalY)){				//ゴールするまで繰り返す

        set_wall(mypos.x,mypos.y);					//壁をセット

        switch(get_nextdir(goalX,goalY,MASK_SEARCH,&glob_nextdir))		//次に行く方向を戻り値とする関数を呼ぶ
        {
            case front:

                result = straight(DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                // straight(SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);		//半区画進む
                break;

            case right:
                if(gObsIsWall[DIREC_LEFT] == 1){
                    // 左に壁があればけつあて
                    doHipAdjust = 1;
                }
                result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
                result = turn(-M_PI_2, TIME_OUT);

                if(doHipAdjust){
                    // けつあて
                    result = straightBack(KETSU_TIME_OUT);
                    // ジャイロのバイアスリセット
                    gGyroBiasResetRequest = 1;
                    while(gGyroBiasResetRequest){
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                    }
                    result = straight(KETSU_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                    doHipAdjust = 0;
                }
                result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//半区画進む
                // turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
                // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                break;

            case left:
                if(gObsIsWall[DIREC_RIGHT] == 1){
                    // 右に壁があればけつあて
                    doHipAdjust = 1;
                }
                result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
                result = turn(M_PI_2, TIME_OUT);
                if(doHipAdjust){
                    // けつあて
                    result = straightBack(KETSU_TIME_OUT);
                    // ジャイロのバイアスリセット
                    gGyroBiasResetRequest = 1;
                    while(gGyroBiasResetRequest){
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                    }
                    result = straight(KETSU_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                    doHipAdjust = 0;
                }
                result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//半区画進む
                // turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
                // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                break;

            case rear:
                if(gObsIsWall[DIREC_FRONT] == 1){
                    // 前に壁があればけつあて
                    doHipAdjust = 1;
                }
                result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
                result = turn(M_PI, TIME_OUT);
                if(doHipAdjust){
                    // けつあて
                    result = straightBack(KETSU_TIME_OUT);
                    // ジャイロのバイアスリセット
                    gGyroBiasResetRequest = 1;
                    while(gGyroBiasResetRequest){
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                    }
                    result = straight(KETSU_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                    doHipAdjust = 0;
                }
                result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);		//半区画進む
                // turn(180,TURN_ACCEL,TURN_SPEED,RIGHT);					//180ターン
                // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,SEARCH_SPEED);
                break;
        }

        // con_wall.enable = TRUE;						//壁制御を有効にする
        // len_mouse = 0;						//進んだ距離をカウントする変数をリセット
        // MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1;			//カウントスタート

        mypos.dir = glob_nextdir;					//方向を更新

        //向いた方向によって自分の座標を更新する
        switch(mypos.dir)
        {
            case north:
                mypos.y++;	//北を向いた時はY座標を増やす
                break;

            case east:
                mypos.x++;	//東を向いた時はX座標を増やす
                break;

            case south:
                mypos.y--;	//南を向いた時はY座標を減らす
                break;

            case west:
                mypos.x--;	//西を向いたときはX座標を減らす
                break;

        }

    }
    set_wall(mypos.x,mypos.y);		//壁をセット
    result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
    // straight(HALF_SECTION,SEARCH_ACCEL,SEARCH_SPEED,0);	
    // 制御終了状態
    gMotorDuty[RIGHT] = 0;
    gMotorDuty[LEFT] = 0;

    gIndicatorValue = 9;
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    gMotorState = MOTOR_OFF;
    gIndicatorValue = 0;

    // ダイアルを初期化
    gObsDial = 0;

}


