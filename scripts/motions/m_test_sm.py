#!/usr/bin/python
from core_tool import *
from state_machine import *
def Help():
  return '''State machine test.
  Usage: test_sm'''
class TLocal:
  tmp= 1
def Run(t,args=()):
  import time,sys

  local= TLocal()
  var1= '- hoge hoge'

  local.start_time= 0
  def GetStartTime():
    local.start_time= int(time.time())  #Changes
    var1= '- HOGE HOGE'  #Not changes

  sm= TStateMachine()
  #sm.Debug= True

  sm.StartState= 'start'
  sm['start']= TFSMState()
  sm['start'].EntryAction= lambda: Print('Hello state machine!',var1)
  sm['start'].NewAction()
  sm['start'].Actions[-1].Condition= lambda: Print("Want to move?") or AskYesNo()
  sm['start'].Actions[-1].NextState= 'count'
  sm['start'].ElseAction.Condition= lambda: True
  sm['start'].ElseAction.Action= lambda: Print('Keep to stay in start\n')
  sm['start'].ElseAction.NextState= 'start'
  sm['start'].ExitAction= lambda: (Print('-->'), GetStartTime())

  sm['count']= TFSMState()
  sm['count'].EntryAction= lambda: Print('Counting...')
  sm['count'].NewAction()
  sm['count'].Actions[-1].Condition= lambda: (int(time.time())-local.start_time)>=3
  sm['count'].Actions[-1].Action= lambda: Print('Hit!: '+str(int(time.time())-local.start_time))
  sm['count'].Actions[-1].NextState= 'stop'
  sm['count'].ElseAction.Condition= lambda: True
  sm['count'].ElseAction.Action= lambda: (Print(str(int(time.time())-local.start_time)), time.sleep(0.2))
  sm['count'].ElseAction.NextState= 'count'

  sm['stop']= TFSMState()
  sm['stop'].EntryAction= lambda: Print('Finishing state machine',var1)
  sm['stop'].ElseAction.Condition= lambda: True
  sm['stop'].ElseAction.NextState= EXIT_STATE

  #sm.Show()
  sm.Run()
