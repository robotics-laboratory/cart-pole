from pydrake.math import cos, sin
from pydrake.systems.framework import BasicVector_, LeafSystem_
from pydrake.systems.scalar_conversion import TemplateSystem

import numpy

@TemplateSystem.define("CartPoleSystem")
def CartPoleSystem_(T):
    """
    Parameters:
      g - gravity
      l – pole length
      
    Input:
      u - target cart acceleration
    
    State:
      x - cart position
      a - pole angle
      v - cart velocity
      w - pole angular velocity
    """
    
    class Impl(LeafSystem_[T]):
        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter=converter)
    
            parameter_default = BasicVector_[T]([9.8, 0.3])
            self.parameter_index = self.DeclareNumericParameter(parameter_default)
    
            self.state_index = self.DeclareContinuousState(4)
            self.DeclareStateOutputPort('q', self.state_index)

            self.input_index = self.DeclareVectorInputPort('u', 1)
            
            self.guard = 0

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)
            
        def DoCalcTimeDerivatives(self, context, derivatives):                
            parameter = context.get_numeric_parameter(self.parameter_index)
            g = parameter[0]
            l = parameter[1]

            q = context.get_continuous_state_vector()
            x = q[0]
            a = q[1]
            v = q[2]
            w = q[3]

            u = self.get_input_port().Eval(context)[0]

            d = derivatives.get_mutable_vector()
            d.SetAtIndex(0, v)
            d.SetAtIndex(1, w)
            d.SetAtIndex(2, u)
            d.SetAtIndex(3, -(u*cos(a) + g*sin(a))/l)
            
            if self.guard > 100:
                return
            
            res = -(u*cos(a) + g*sin(a))/l
            print(f'{self.guard} q={q} u={u} a={a} g={g} l={l} res={res}')
            self.guard += 1

        def CreateContext(self, config, q):
            context = self.CreateDefaultContext()
            context.SetTime(0)

            params = context.get_mutable_numeric_parameter(self.parameter_index)
            params.SetFromVector(
                numpy.array([config.gravity, config.pole_length])
            )
            
            context.SetContinuousState(q)

            return context
                      

    return Impl

CartPoleSystem = CartPoleSystem_[float]