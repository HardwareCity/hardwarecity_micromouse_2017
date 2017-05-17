//
// Created by Rui Filipe de Sousa Martins on 14/05/17.
//

#ifndef SRC_STATE_MACHINES_H
#define SRC_STATE_MACHINES_H
    // Mouse States
    #define STATE_MOUSE_WAITTING_TO_START 10
    #define STATE_MOUSE_AVOIDING_COLISION 20
    #define STATE_MOUSE_WALKING 30
//    #define STATE_MOUSE_GOING_TO_CHEESE 2
    #define STATE_MOUSE_ON_CHEESE 40
//    #define STATE_MOUSE_BACK_TO_HOME 4
    #define STATE_MOUSE_ALL_DONE 50
    #define STATE_MOUSE_ABORTED 60
    #define STATE_MOUSE_TIMEOUT 70
    #define STATE_MOUSE_SEARCH_BEACON 80


    //TODO: Usas valores repetidos nos estados?!
    // (Estou a ir ou a voltar do farol? Se tiver a voltar, desligo o sensor do chão e o farol)
    #define STATE_JOURNEY_GOING_TO_CHEESE 1
    #define STATE_JOURNEY_RETURN_TO_HOME 2

#endif //SRC_STATE_MACHINES_H
