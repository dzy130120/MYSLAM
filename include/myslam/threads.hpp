#pragma once
#include<pthread.h>
#include<iostream>
namespace my_slam
{
    typedef void* (*threadFunType)(void* );
    class mythread
    {
    public:
        mythread();
        mythread(threadFunType thread);
        //~mythread();
        pthread_t*  thread_ID;
        threadFunType threadfun;


    };
}



