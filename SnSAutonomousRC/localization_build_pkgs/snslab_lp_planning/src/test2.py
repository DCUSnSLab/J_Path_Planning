
import numpy as np
from enum import Enum
from threading import Thread
import rospy
from std_msgs.msg import Int64
import time

arr = [[10,20],[30,40],[50,60]]
arr2 = [[10,20],[50,40]]
#arr2 = [10,20]
arr = np.array(arr)
arr2 = np.array(arr2)

#print(arr==arr2)
#if (arr == arr2).all()==True:
    #print(1)


a = np.array([[15, 8, 12], [11, 7, 3]])
#print(np.where(a > 10)) # (array([0, 0, 1]), array([0, 2, 0]))


from sympy import *
import math

x,y, = symbols('x y')

#print(solve( [ Eq(x**2 - y**2, 0), Eq(2*x**2 - x*y + y**2, 8) ], [x,y] ))

b = [[1,2], [3,4], [5,6]]
c = [3,4,5,6]

print(np.arctan(1))
print(math.atan((4-3)/(6-5)))


'''
class AlgorithmType(Enum):
    algorithmFGM = 0
    DisparityExtender = 1


class test():
    def __init__(self):
        self.name = 'algorithmFGM'
        self.i=1

    def test(self):
        while True:
            print('FGM : 0, DisparityExtender : 1')
            i = int(input())
            self.name = AlgorithmType(i).name
            print(self.i)
            #print(AlgorithmType(i).name)
            #print(type(AlgorithmType(i).name))

    def test2(self):
        while True:
            self.i+=1




if __name__ == '__main__':
    t = test()
    threads = []
    th1 = Thread(target=t.test)
    th2 = Thread(target=t.test2)
    th1.start()
    th2.start()
    th1.join()
    th2.join()
'''



'''
import threading


# CounterThread
class CounterThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, name='Timer Thread')

    # CounterThread가 실행하는 함수
    def run(self):
        global totalCount

        # 2,500,000번 카운트 시작
        for _ in range(2500000):
            totalCount += 1
        print('2,500,000번 카운팅 끝!')


if __name__ == '__main__':
    # 전역 변수로 totalCount를 선언
    global totalCount
    totalCount = 0

    # totalCount를 1씩 더하는
    # Counter Thread를 4개 만들어서 동작시킨다.
    for _ in range(4):
        timerThread = CounterThread()
        timerThread.start()

    print('모든 Thread들이 종료될 때까지 기다린다.')
    mainThread = threading.currentThread()
    for thread in threading.enumerate():
        # Main Thread를 제외한 모든 Thread들이
        # 카운팅을 완료하고 끝날 때 까지 기다린다.
        if thread is not mainThread:
            thread.join()

    print('totalCount = ' + str(totalCount))
'''