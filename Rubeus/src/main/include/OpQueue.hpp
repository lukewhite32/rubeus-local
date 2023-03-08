#pragma once

#include <typeinfo>


struct RobotOperation {
  const std::type_info* me;
};


class OpQueue {
    std::vector <RobotOperation*> queue;
public:
    void Push (RobotOperation* thang){
        queue.push_back (thang);
    }

    template <typename T>
    T* New() {
        static_assert(std::is_base_of<RobotOperation, T>());
        T* ret = new T;
        ret -> me = &typeid(T);
        return ret;
    }

    void Finish() {
        delete queue[0];
        queue.erase(queue.begin());
    }

    RobotOperation* Get() {
        return queue[0];
    }

    template <typename T>
    T Get() {
        static_assert(std::is_base_of<RobotOperation, T>());
        RobotOperation* thang = Get();
        if (typeid(T) != *thang -> me){ // You're trying to get the wrong thang
            throw std::runtime_error("Well shoot you done requestified the wrong type");
        }
        return *((T*)thang);
    }

    const std::type_info* TopType(){
        return Get() -> me;
    }

    template <typename T>
    bool Is(){
        return typeid(T) == *TopType();
    }

    bool NotEmpty(){
        return queue.size() > 0;
    }

    operator bool() const{
        return queue.size() > 0;
    }

    auto Size(){
        return queue.size();
    }

    void Clear(){
        while (NotEmpty()){
            Finish();
        }
    }
};