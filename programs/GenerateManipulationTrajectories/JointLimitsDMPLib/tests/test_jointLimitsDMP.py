from JointLimitsDMPLib import jointLimitsDMP
import numpy as np
import math
import pytest
def test_jointLimitsDMP():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60]
    qmax = [25,65]
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax)
    assert jointLimitsDMP1.dt == dt
    assert jointLimitsDMP1.execution_time == execution_time
    assert jointLimitsDMP1.n_features == n_features
    assert jointLimitsDMP1.q_min.size != 0
    assert jointLimitsDMP1.q_max.size != 0


def test_SameDimensions():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60]
    qmax = [25,65]
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax)
    y_demo = np.array([])
    x0 = np.zeros(2)
    g = np.zeros(2)
    with pytest.raises(Exception) as exception_info:
        correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
        assert str(exception_info.value) == 'demo.rows != x0.rows'
    
    g = np.zeros(3)
    with pytest.raises(Exception) as exception_info:
        correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
        assert str(exception_info.value) == 'demo.rows != g.rows'
      
    g = np.zeros(2)
    time = np.linspace(0,execution_time, math.ceil(execution_time/dt))
    y_demo1 = np.sin(time*3*np.pi/2)*20*np.pi/180
    y_demo1 = np.vstack((y_demo1,-np.sin(time*3*np.pi/2)*50*np.pi/180))
    with pytest.raises(Exception) as exception_info:
        correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
        assert str(exception_info.value) == "demo.columns != execution_time/dt+1"



def test_imitateDemoInsideBounds():
    dt = 0.001
    execution_time = 1.0
    n_features = 10
    qmin = [-40,-60, -55]
    qmax = [25,65, 50]
    jointLimitsDMP1 = jointLimitsDMP.JointLimitsDMP(dt, execution_time, n_features, qmin, qmax)
    y_demo = np.array([])
    x0 = np.zeros(3)
    x0 =  np.expand_dims(x0, axis=1)

    g = np.zeros(3)
    g[0] = 10*np.pi/180
    g[1] = -10*np.pi/180
    g[2] = 20*np.pi/180
    print(g)
    g =  np.expand_dims(g, axis=1)
    time = np.linspace(0,execution_time, math.ceil(execution_time/dt)+1)
    y_demo = np.sin(time*3*np.pi/2)*20*np.pi/180
    y_demo = np.vstack((y_demo,-np.sin(time*3*np.pi/2)*50*np.pi/180))
    y_demo = np.vstack((y_demo,-np.sin(time*6*np.pi/2)*30*np.pi/180))
    print(y_demo.shape)
    correct, trajectory = jointLimitsDMP1.imitateDemoInsideBounds(y_demo, x0, g)
    jointLimitsDMP1.plotDMPS(y_demo,trajectory)
    assert correct == True
    

    
    