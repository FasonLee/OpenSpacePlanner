/***********************************
 * File Name   : Timer.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/
#ifndef OPEN_SPACE_UTILS_TIMER_H
#define OPEN_SPACE_UTILS_TIMER_H

#include <string>
#include <cinttypes>
#include <ctime>
#include <sys/time.h>

namespace open_space_utils
{
    class Timer
    {
      public:

        inline static std::string getReadableTimestampUS(bool fast = false,
            int zone_offset = 8)
        {
            char            fmt[64], buf[64];
            struct timeval  tv;
            struct tm       tm;

            gettimeofday(&tv, NULL);
            auto getTime = fast ? gmtime : localtime;
            if (auto res = getTime(&tv.tv_sec); res != nullptr)
            {
                tm = *res;
                if (fast)
                {
                    tm.tm_hour += zone_offset;
                    if (tm.tm_hour > 23)
                        tm.tm_hour -= 24;
                }
                strftime(fmt, sizeof fmt, "%H.%M.%S.%%06u", &tm);
                snprintf(buf, sizeof buf, fmt, tv.tv_usec);
                return std::string(buf);
            }
            else
            {	
                time_t rawtime;
                struct tm *info;
                char buffer[80];
                time( &rawtime );
                info = localtime( &rawtime );
                strftime(buffer, 80, "%H_%M_%S", info);
                return std::string(buffer);
            }
        }

        inline static std::string getReadableTimestampMS(bool fast = false,
            int zone_offset = 8)
        {
            char            fmt[64], buf[64];
            struct timeval  tv;
            struct tm       tm;

            gettimeofday(&tv, NULL);
            auto getTime = fast ? gmtime : localtime;
            if (auto res = getTime(&tv.tv_sec); res != nullptr)
            {
                tm = *res;
                if (fast)
                {
                    tm.tm_hour += zone_offset;
                    if (tm.tm_hour > 23)
                        tm.tm_hour -= 24;
                }
                strftime(fmt, sizeof fmt, "%H.%M.%S.%%03u", &tm);
                snprintf(buf, sizeof buf, fmt, tv.tv_usec / 1000);
                return std::string(buf);
            }
            else
            {	
                time_t rawtime;
                struct tm *info;
                char buffer[80];
                time( &rawtime );
                info = localtime( &rawtime );
                strftime(buffer, 80, "%H_%M_%S", info);
                return std::string(buffer);
            }
        }
        
        /*
        获取系统启动后的运行时间(微秒)
        */
        inline static uint64_t getSystemTimestampUS()
        {
            struct timespec ts;        
            //该函数是用于获取特定 时钟的时间，常用如下4种时钟
            /*
            CLOCK_REALTIME                  //系统当前时间，从1970年1.1日算起
            CLOCK_MONOTONIC                 //系统的启动后运行时间，不能被设置
            CLOCK_PROCESS_CPUTIME_ID        //本进程运行时间
            CLOCK_THREAD_CPUTIME_ID         //本线程运行时间            
            */    
            clock_gettime(CLOCK_MONOTONIC, &ts);
            return (uint64_t)(ts.tv_sec * 1e6 + ts.tv_nsec * 0.001);
        }

        /*
        returns the wall-clock time with µs precision
        相对于1970年1月1日的微秒数
        系统时间(wall clock time, elapsed time). 是指一段程序从运行到终止，
        系统时钟走过的时间。一般来说，系统时间都是要大于CPU时间的。
        */
        inline static uint64_t getTimestampUS()
        {
            struct timeval tv;
            //返回的当前时间tv.tv_sec 是从1970年1月1日0 点开始的 “秒”数
            gettimeofday(&tv, NULL);
            return (uint64_t)(tv.tv_sec * 1e6 + tv.tv_usec);
        }

        /*
        获取系统启动后的运行时间(毫秒)
        */
        inline static uint64_t getSystemTimestampMS()
        {
            return getSystemTimestampUS() / 1000;
        }

        /*
        相对于1970年1月1日的毫秒数
        */
        inline static uint64_t getTimestampMS()
        {
            return getTimestampUS() / 1000;
        }
    };
}

#endif // OPEN_SPACE_UTILS_TIMER_H