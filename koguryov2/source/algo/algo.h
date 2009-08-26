//###########################################################################
//
// FILE:   algo.h
//
// TITLE:  KOGURYO Mouse algorithm header file.
//
//###########################################################################
// $Release Date: 2006.12.27 $
//###########################################################################

#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

extern void InitSearchQueueList(void);
extern void InsertSearchQueueList(Uint16 pos, Uint16 weight);
extern int16 PopSearchQueueList(void);
extern void DelSearchQueueList(void);
extern void InitAlgorithmVariable(void);
extern void InitWeight(void);
extern void InitAlgorithm(void);
extern void WriteMazeWeight(Uint16 NowPos);
extern void Algorithm(Uint16 WallInfo);
extern void KnowBlockPathMake(Uint16 Position, Uint16 MouseDir);
extern void RunPathMake(void);

#endif

