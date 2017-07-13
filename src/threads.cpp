#include<test1.hpp>
#include<pthread.h>
using namespace my_thread;
mythread::mythread(threadFunType thread):threadfun(thread)
{
    thread_ID = new pthread_t();
    int ret = pthread_create(thread_ID, NULL, threadfun, NULL);
    if(!ret)
    {
        std::cout<<"thread_ID: "<<*thread_ID<<std::endl;
    }
    else
    {
        std::cout<<"creat new thread fail"<<std::endl;
    }
}



