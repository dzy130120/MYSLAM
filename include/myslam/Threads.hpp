#pragma once
#include<myslam/CommonInclude.hpp>

namespace myslam
{

    template<class T, class R> class THREAD
    {
    public:
        typedef std::shared_ptr<THREAD> Ptr;
        typedef void (R::* fptr_) ();
        //THREAD(){};
        THREAD(T object_, fptr_ fun)
        {
            //*(object_->fun)();
            //cout<<object_<<endl;
             t2 = new std::thread(threadfun, object_, fun);
        }


        static void threadfun(T obj_, fptr_ f)
        {
            ((*obj_).*f)();
        }

        std::thread* t2;


    };

}



