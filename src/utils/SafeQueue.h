#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

// A threadsafe-queue. - https://stackoverflow.com/questions/15278343/c11-thread-safe-queue
template <class T>
class SafeQueue
{
public:
  SafeQueue(void)
    : q()
    , m()
    , c()
  {}

  ~SafeQueue(void)
  {}

  // Add an element to the queue.
  void enqueue(T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q.push(t);
    c.notify_one();
  }



  // Get the "front"-element.
  // If the queue is empty, wait till a element is avaiable.
  T dequeue(void)
  {
    std::unique_lock<std::mutex> lock(m);
    while(q.empty())
    {
      // release lock as long as the wait and reaquire it afterwards.
      c.wait(lock);
    }
    T val = q.front();
    q.pop();
    return val;
  }

  ////// added my mpkuse

  // return size
  int size()
  {
    std::lock_guard<std::mutex> lock(m);
    int n = q.size();
    c.notify_one();
    return n;
  }

  // Add an element to the queue.
  void push(T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q.push(t);
    c.notify_one();
  }

  // Get the "front"-element.
  // If the queue is empty, wait till a element is avaiable.
  T pop(void)
  {
    std::unique_lock<std::mutex> lock(m);
    while(q.empty())
    {
      // release lock as long as the wait and reaquire it afterwards.
      c.wait(lock);
    }
    T val = q.front();
    q.pop();
    return val;
  }

  // most recently added element
  T back()
  {
    assert( size() > 0 && "SafeQueue: You tried to SafeQueue::front(), but the queue was empty. This operation is only allowed when there are elements in the queue.  ");
    std::lock_guard<std::mutex> lock(m);
    T val = q.back();
    c.notify_one();
    return val;
  }

  // 1st element in queue
  T front()
  {
    assert( size() > 0 && "SafeQueue: You tried to SafeQueue::front(), but the queue was empty. This operation is only allowed when there are elements in the queue.  ");
    std::lock_guard<std::mutex> lock(m);
    T val = q.front();
    c.notify_one();
    return val;
  }

private:
  std::queue<T> q;
  mutable std::mutex m;
  std::condition_variable c;
};
