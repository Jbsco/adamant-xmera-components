--------------------------------------------------------------------------------
-- Stepper_Motor_Controller Tests Spec
--------------------------------------------------------------------------------

-- Integration tests for the Stepper Motor Controller component.
package Stepper_Motor_Controller_Tests.Implementation is

   -- Test data and state:
   type Instance is new Stepper_Motor_Controller_Tests.Base_Instance with private;
   type Class_Access is access all Instance'Class;

private
   -- Fixture procedures:
   overriding procedure Set_Up_Test (Self : in out Instance);
   overriding procedure Tear_Down_Test (Self : in out Instance);

   -- Validate motor step command output against reference Python scenarios.
   overriding procedure Test (Self : in out Instance);

   -- Test data and state:
   type Instance is new Stepper_Motor_Controller_Tests.Base_Instance with record
      null;
   end record;
end Stepper_Motor_Controller_Tests.Implementation;
