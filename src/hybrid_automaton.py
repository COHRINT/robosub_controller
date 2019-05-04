import numpy as np
import time
import sys
import copy

class Automaton():
    def __init__(self):
        pass
        #  phy_sys=[{'name':'x1','output':'a','trans':[{'state':'x3','sigma':'2'},
        #      {'state':'x2','sigma':'1'}]},{'name':'x2','output':'b','trans':[{'state':'x4','sigma':'1'}]},
        #      {'name':'x3','output':'a','trans':[{'state':'x4','sigma':'1'}]},{'name':'x4','output':'c','trans':[{'state':'x1','sigma':'1'},
        #      {'state':'x2','sigma':'2'}]}]
        #  buchi=[{'name':'q0','type':'init','trans':[{'state':'q1','output':'c'},{'state':'q0','output':'a'}]},
        #          {'name':'q1','type':'accept','trans':[{'state':'q1','output':1}]}]


    def product(self,phy_sys,buchi):
        combined=[]
        for state in phy_sys:
            for buc_state in buchi:
                new={}
                new['name1']=state['name']
                new['name2']=buc_state['name']
                new['output']=state['output']
                new['type']=buc_state['type']
                #  new['location']=state['location']
                new['trans']=[]
                combined.append(new)
        #  print combined
        for state in combined:
            output=state['output']
            name1=state['name1']
            name2=state['name2']
            for buc_state in buchi:
                if buc_state['name']==name2:
                    for trans in buc_state['trans']:
                        if (output in trans['output']) or (trans['output']=='1'):
                            trans_name2=trans['state']
                            for phy_state in phy_sys:
                                if phy_state['name']==name1:
                                    for phy_trans in phy_state['trans']:
                                        state['trans'].append({'name1':phy_trans['state'],'name2':trans_name2,
                                            'sigma':phy_trans['sigma']})
        #  print combined
        return combined

    def backward_reachability(self,automata):
        current_states=[]
        for state in automata:
            if state['type']=='accept':
                current_states.append(state['name1']+state['name2'])
        #  print current_states
        #  raw_input()
        #  sys.exit()
        #  while len(current_states)>0:
        previous_states=[]
        while current_states!=previous_states:
            previous_states=copy.copy(current_states)
            new_states=[]
            for state in automata:
                trigger=False
                for trans in state['trans']:
                    if (trans['name1']+trans['name2']) in current_states:
                        trigger=True
                        new_states.append(state['name1']+state['name2'])
                    elif trigger:
                        state['trans'].remove(trans)
            new_states=list(set(new_states))
            #  for state in current_states:
            #      if state in new_states:
            #          new_states.remove(state)
            current_states=copy.copy(new_states)
            #  print current_states
            #  raw_input()
        #  print automata
        return automata



    def planner(self,current,automata):
        for state in automata:
            if len(current)==5:
                name1=current[0:3]
                name2=current[3:]
            else:
                name1=current[0:2]
                name2=current[2:]
            if (state['name1']==name1) and (state['name2']==name2):
                if state['type']=='accept':
                    print "You're done, congrats"
                    sys.exit()
                if len(state['trans'])>1:
                    transition=np.random.choice(range(len(state['trans'])))
                    transition_state=state['trans'][transition]['name1']+state['trans'][transition]['name2']
                elif len(state['trans'])==0:
                    print "You get nothing! You lose! Good day, sir!"
                    sys.exit()
                else:
                    transition_state=state['trans'][0]['name1']+state['trans'][0]['name2']
        print transition_state
        #  for state in automata:
        #      if (state['name1']==transition_state[0:2]) and (state['name2']==transition_state[2:])
        #          point=state['location']
        #  return transition_state,point
        return transition_state

if __name__ == '__main__':
    a=Automaton()
    #  phy_sys=[{'name':'x1','output':'a',
    #          'trans':[{'state':'x3','sigma':'2'},
    #              {'state':'x2','sigma':'1'}]},
    #          {'name':'x2','output':'b',
    #          'trans':[{'state':'x4','sigma':'1'}]},
    #          {'name':'x3','output':'a',
    #          'trans':[{'state':'x4','sigma':'1'}]},
    #          {'name':'x4','output':'c',
    #          'trans':[{'state':'x1','sigma':'1'},
    #                  {'state':'x2','sigma':'2'}]}]
    #  buchi=[{'name':'q0','type':'init',
    #          'trans':[{'state':'q1','output':'c'},
    #                  {'state':'q0','output':'a'}]},
    #          {'name':'q1','type':'accept',
    #          'trans':[{'state':'q1','output':'1'}]}]
    buchi=[{'name':'q0','type':'init',
            'trans':[{'state':'q0','output':'l'},
                    {'state':'q1','output':'g'}]},
            {'name':'q1','type':'regular',
            'trans':[{'state':'q1','output':'grl'},
                    {'state':'q2','output':'d'}]},
            {'name':'q2','type':'regular',
            'trans':[{'state':'q2','output':'grld'},
                    {'state':'q3','output':'s'}]},
            {'name':'q3','type':'accept',
            'trans':[{'state':'q3','output':'1'}]}]
    phy_sys=[{'name':'x1','output':'s','location':[0,0,0],
            'trans':[{'state':'x2','sigma':'f'},
                    {'state':'x5','sigma':'r'},
                    {'state':'x1','sigma':'s'},
                    {'state':'x13','sigma':'d'}]},
            {'name':'x2','output':'s','location':[5,0,0],
            'trans':[{'state':'x3','sigma':'f'},
                    {'state':'x6','sigma':'r'},
                    {'state':'x1','sigma':'b'},
                    {'state':'x2','sigma':'s'},
                    {'state':'x14','sigma':'d'}]},
            {'name':'x3','output':'s','location':[10,0,0],
            'trans':[{'state':'x4','sigma':'f'},
                    {'state':'x7','sigma':'r'},
                    {'state':'x2','sigma':'b'},
                    {'state':'x3','sigma':'s'},
                    {'state':'x15','sigma':'d'}]},
            {'name':'x4','output':'s','location':[15,0,0],
            'trans':[{'state':'x3','sigma':'b'},
                    {'state':'x8','sigma':'r'},
                    {'state':'x4','sigma':'s'},
                    {'state':'x16','sigma':'d'}]},
            {'name':'x5','output':'s','location':[0,5,0],
            'trans':[{'state':'x1','sigma':'l'},
                    {'state':'x9','sigma':'r'},
                    {'state':'x5','sigma':'s'},
                    {'state':'x17','sigma':'d'},
                    {'state':'x6','sigma':'f'}]},
            {'name':'x6','output':'s','location':[5,5,0],
            'trans':[{'state':'x2','sigma':'l'},
                    {'state':'x10','sigma':'r'},
                    {'state':'x6','sigma':'s'},
                    {'state':'x5','sigma':'b'},
                    {'state':'x7','sigma':'f'}]},
            {'name':'x7','output':'s','location':[10,5,0],
            'trans':[{'state':'x3','sigma':'l'},
                    {'state':'x11','sigma':'r'},
                    {'state':'x7','sigma':'s'},
                    {'state':'x19','sigma':'d'},
                    {'state':'x6','sigma':'b'},
                    {'state':'x8','sigma':'f'}]},
            {'name':'x8','output':'s','location':[15,5,0],
            'trans':[{'state':'x4','sigma':'l'},
                    {'state':'x12','sigma':'r'},
                    {'state':'x8','sigma':'s'},
                    {'state':'x20','sigma':'d'},
                    {'state':'x7','sigma':'b'}]},
            {'name':'x9','output':'s','location':[0,10,0],
            'trans':[{'state':'x5','sigma':'l'},
                    {'state':'x9','sigma':'s'},
                    {'state':'x10','sigma':'f'},
                    {'state':'x21','sigma':'d'}]},
            {'name':'x10','output':'s','location':[5,10,0],
            'trans':[{'state':'x6','sigma':'l'},
                    {'state':'x10','sigma':'s'},
                    {'state':'x11','sigma':'f'},
                    {'state':'x9','sigma':'b'},
                    {'state':'x22','sigma':'d'}]},
            {'name':'x11','output':'s','location':[10,10,0],
            'trans':[{'state':'x7','sigma':'l'},
                    {'state':'x11','sigma':'s'},
                    {'state':'x12','sigma':'f'},
                    {'state':'x10','sigma':'b'},
                    {'state':'x23','sigma':'d'}]},
            {'name':'x12','output':'s','location':[15,10,0],
            'trans':[{'state':'x8','sigma':'l'},
                    {'state':'x12','sigma':'s'},
                    {'state':'x11','sigma':'b'},
                    {'state':'x24','sigma':'d'}]},
            {'name':'x13','output':'l','location':[0,0,5],
            'trans':[{'state':'x14','sigma':'f'},
                    {'state':'x17','sigma':'r'},
                    {'state':'x13','sigma':'s'},
                    {'state':'x1','sigma':'u'}]},
            {'name':'x14','output':'l','location':[5,0,5],
            'trans':[{'state':'x15','sigma':'f'},
                    {'state':'x14','sigma':'s'},
                    {'state':'x13','sigma':'b'},
                    {'state':'x2','sigma':'u'}]},
            {'name':'x15','output':'r','location':[10,0,5],
            'trans':[{'state':'x16','sigma':'f'},
                    {'state':'x19','sigma':'r'},
                    {'state':'x14','sigma':'b'},
                    {'state':'x15','sigma':'s'},
                    {'state':'x3','sigma':'u'}]},
            {'name':'x16','output':'r','location':[15,0,5],
            'trans':[{'state':'x15','sigma':'b'},
                    {'state':'x20','sigma':'r'},
                    {'state':'x16','sigma':'s'},
                    {'state':'x4','sigma':'u'}]},
            {'name':'x17','output':'l','location':[0,5,5],
            'trans':[{'state':'x13','sigma':'l'},
                    {'state':'x21','sigma':'r'},
                    {'state':'x17','sigma':'s'},
                    {'state':'x5','sigma':'u'},
                    {'state':'x18','sigma':'f'}]},
            {'name':'x18','output':'g','location':[5,5,5],
            'trans':[{'state':'x18','sigma':'s'},
                    {'state':'x17','sigma':'b'},
                    {'state':'x19','sigma':'f'}]},
            {'name':'x19','output':'r','location':[10,5,5],
            'trans':[{'state':'x15','sigma':'l'},
                    {'state':'x23','sigma':'r'},
                    {'state':'x19','sigma':'s'},
                    {'state':'x7','sigma':'u'},
                    {'state':'x18','sigma':'b'},
                    {'state':'x20','sigma':'f'}]},
            {'name':'x20','output':'d','location':[15,5,5],
            'trans':[{'state':'x16','sigma':'l'},
                    {'state':'x24','sigma':'r'},
                    {'state':'x20','sigma':'s'},
                    {'state':'x8','sigma':'u'},
                    {'state':'x19','sigma':'b'}]},
            {'name':'x21','output':'l','location':[0,10,5],
            'trans':[{'state':'x17','sigma':'l'},
                    {'state':'x21','sigma':'s'},
                    {'state':'x22','sigma':'f'},
                    {'state':'x9','sigma':'u'}]},
            {'name':'x22','output':'l','location':[5,10,5],
            'trans':[{'state':'x22','sigma':'s'},
                    {'state':'x23','sigma':'f'},
                    {'state':'x21','sigma':'b'},
                    {'state':'x10','sigma':'u'}]},
            {'name':'x23','output':'r','location':[10,10,5],
            'trans':[{'state':'x19','sigma':'l'},
                    {'state':'x23','sigma':'s'},
                    {'state':'x24','sigma':'f'},
                    {'state':'x22','sigma':'b'},
                    {'state':'x11','sigma':'u'}]},
            {'name':'x24','output':'r','location':[15,10,5],
            'trans':[{'state':'x20','sigma':'l'},
                    {'state':'x24','sigma':'s'},
                    {'state':'x23','sigma':'b'},
                    {'state':'x12','sigma':'u'}]},
            ]
    automata=a.product(phy_sys,buchi)
    #  print automata
    automata=a.backward_reachability(automata)
    current='x21q0'
    #  current='x1q0'
    for i in range(100):
        current=a.planner(current,automata)
