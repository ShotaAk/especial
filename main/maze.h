#ifndef MAZE_H 
#define MAZE_H

extern void initMaze(void);
extern void search(const int goalX, const int goalY, const int slalomEnable, 
        const int goHomeEnable);
extern void run(const int goalX, const int goalY, const int slalomEnable, 
        const int goHomeEnable);

#endif
