--------------------------------------------------------------------------------
-- Average_Mimu_Data Tests Body
--------------------------------------------------------------------------------

with Basic_Assertions; use Basic_Assertions;
with Acc_Data;
with Imu_Sensor_Body;
with Packed_F32x3.Assertion; use Packed_F32x3.Assertion;
with Average_Mimu_Data_Parameters;
with Parameter_Enums.Assertion;
use Parameter_Enums.Parameter_Update_Status;
use Parameter_Enums.Assertion;

package body Average_Mimu_Data_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;

      -- Call component init here.
      Self.Tester.Component_Instance.Init;

      -- Call the component set up method that the assembly would normally call.
      Self.Tester.Component_Instance.Set_Up;
   end Set_Up_Test;

   overriding procedure Tear_Down_Test (Self : in out Instance) is
   begin
      -- Free component heap:
      Self.Tester.Component_Instance.Destroy;
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Run algorithm to ensure integration is sound.
   overriding procedure Test (Self : in out Instance) is
      T : Component.Average_Mimu_Data.Implementation.Tester.Instance_Access renames Self.Tester;
      Params : Average_Mimu_Data_Parameters.Instance;

      -- Uniform AccData: all 120 packets have the same gyro and accel values.
      -- With timeDelta large enough, all packets are included in the average,
      -- so the average equals the uniform value.
      Test_Acc_Data : constant Acc_Data.T := (
         Acc_Pkts => [others => (
            Meas_Time => 1_000_000_000, -- 1 second in nanoseconds
            Gyro_B => [1.0, 2.0, 3.0],
            Accel_B => [4.0, 5.0, 6.0]
         )]
      );
   begin
      -----------------------------------------------------------------------
      -- Set parameters: identity DCM, large time window
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Time_Delta ((Value => 1.0e10))), Success);
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      -----------------------------------------------------------------------
      -- Test Case 1: Identity DCM - output should equal input average
      -----------------------------------------------------------------------
      T.Acc_Data_T_Send (Test_Acc_Data);

      -- Verify data product produced:
      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 1);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 1);

      -- Check output values match expected (identity DCM, uniform data):
      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (1);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [1.0, 2.0, 3.0], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [4.0, 5.0, 6.0], Epsilon => 0.0001);
      end;

      -----------------------------------------------------------------------
      -- Test Case 2: 90-degree Z rotation DCM
      -- DCM = [0, -1, 0; 1, 0, 0; 0, 0, 1]
      -- DCM * [1,2,3] = [-2, 1, 3]
      -- DCM * [4,5,6] = [-5, 4, 6]
      -----------------------------------------------------------------------
      Parameter_Update_Status_Assert.Eq (T.Stage_Parameter (
         Params.Dcm_Pltf_To_Bdy ([
            0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0
         ])), Success);
      Parameter_Update_Status_Assert.Eq (T.Update_Parameters, Success);

      T.Acc_Data_T_Send (Test_Acc_Data);

      -- Verify second data product produced:
      Natural_Assert.Eq (T.Data_Product_T_Recv_Sync_History.Get_Count, 2);
      Natural_Assert.Eq (T.Imu_Body_Data_History.Get_Count, 2);

      -- Check output values match expected (rotated by DCM):
      declare
         Output : constant Imu_Sensor_Body.T := T.Imu_Body_Data_History.Get (2);
      begin
         Packed_F32x3_Assert.Eq (Output.Ang_Vel_Body, [-2.0, 1.0, 3.0], Epsilon => 0.0001);
         Packed_F32x3_Assert.Eq (Output.Accel_Body, [-5.0, 4.0, 6.0], Epsilon => 0.0001);
      end;
   end Test;

end Average_Mimu_Data_Tests.Implementation;
