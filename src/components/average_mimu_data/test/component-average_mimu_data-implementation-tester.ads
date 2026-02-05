--------------------------------------------------------------------------------
-- Average_Mimu_Data Component Tester Spec
--------------------------------------------------------------------------------

-- Includes:
with Component.Average_Mimu_Data_Reciprocal;
with Printable_History;
with Sys_Time.Representation;
with Data_Product.Representation;
with Data_Product;
with Imu_Sensor_Body.Representation;

-- Averages MIMU accelerometer and gyro data within a configurable time window and
-- transforms to the spacecraft body frame.
package Component.Average_Mimu_Data.Implementation.Tester is

   use Component.Average_Mimu_Data_Reciprocal;
   -- Invoker connector history packages:
   package Sys_Time_T_Return_History_Package is new Printable_History (Sys_Time.T, Sys_Time.Representation.Image);
   package Data_Product_T_Recv_Sync_History_Package is new Printable_History (Data_Product.T, Data_Product.Representation.Image);

   -- Data product history packages:
   package Imu_Body_Data_History_Package is new Printable_History (Imu_Sensor_Body.T, Imu_Sensor_Body.Representation.Image);

   -- Component class instance:
   type Instance is new Component.Average_Mimu_Data_Reciprocal.Base_Instance with record
      -- The component instance under test:
      Component_Instance : aliased Component.Average_Mimu_Data.Implementation.Instance;
      -- Connector histories:
      Sys_Time_T_Return_History : Sys_Time_T_Return_History_Package.Instance;
      Data_Product_T_Recv_Sync_History : Data_Product_T_Recv_Sync_History_Package.Instance;
      -- Data product histories:
      Imu_Body_Data_History : Imu_Body_Data_History_Package.Instance;
   end record;
   type Instance_Access is access all Instance;

   ---------------------------------------
   -- Initialize component heap variables:
   ---------------------------------------
   procedure Init_Base (Self : in out Instance);
   procedure Final_Base (Self : in out Instance);

   ---------------------------------------
   -- Test initialization functions:
   ---------------------------------------
   procedure Connect (Self : in out Instance);

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- The system time is retrieved via this connector.
   overriding function Sys_Time_T_Return (Self : in out Instance) return Sys_Time.T;
   -- The data product invoker connector
   overriding procedure Data_Product_T_Recv_Sync (Self : in out Instance; Arg : in Data_Product.T);

   -----------------------------------------------
   -- Data product handler primitives:
   -----------------------------------------------
   -- Description:
   --    Data products for the Average Mimu Data component.
   -- Averaged IMU sensor data (angular velocity and acceleration) in spacecraft body
   -- frame.
   overriding procedure Imu_Body_Data (Self : in out Instance; Arg : in Imu_Sensor_Body.T);

   -----------------------------------------------
   -- Special primitives for aiding in the staging,
   -- fetching, and updating of parameters
   -----------------------------------------------
   -- Stage a parameter value within the component
   not overriding function Stage_Parameter (Self : in out Instance; Par : in Parameter.T) return Parameter_Update_Status.E;
   -- Fetch the value of a parameter with the component
   not overriding function Fetch_Parameter (Self : in out Instance; Id : in Parameter_Types.Parameter_Id; Par : out Parameter.T) return Parameter_Update_Status.E;
   -- Ask the component to validate all parameters. This will call the
   -- Validate_Parameters subprogram within the component implementation,
   -- which allows custom checking of the parameter set prior to updating.
   not overriding function Validate_Parameters (Self : in out Instance) return Parameter_Update_Status.E;
   -- Tell the component it is OK to atomically update all of its
   -- working parameter values with the staged values.
   not overriding function Update_Parameters (Self : in out Instance) return Parameter_Update_Status.E;

end Component.Average_Mimu_Data.Implementation.Tester;
