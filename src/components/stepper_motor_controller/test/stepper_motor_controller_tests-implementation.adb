--------------------------------------------------------------------------------
-- Stepper_Motor_Controller Tests Body
--------------------------------------------------------------------------------

with Ada.Numerics;
with Ada.Real_Time;
with Basic_Assertions; use Basic_Assertions;
with Component.Stepper_Motor_Controller.Implementation;
with Component.Stepper_Motor_Controller.Implementation.Tester;
with Interfaces; use Interfaces;
with Motor_Step_Command;
with Parameter_Enums; use Parameter_Enums;
with Data_Product_Enums;
with Stepper_Motor_Controller_Parameters;
with Sys_Time;

package body Stepper_Motor_Controller_Tests.Implementation is

   -------------------------------------------------------------------------
   -- Fixtures:
   -------------------------------------------------------------------------

   overriding procedure Set_Up_Test (Self : in out Instance) is
   begin
      -- Allocate heap memory to component:
      Self.Tester.Init_Base;

      -- Make necessary connections between tester and component:
      Self.Tester.Connect;
      -- Component Init/Set_Up performed per scenario in test body.
   end Set_Up_Test;

   overriding procedure Tear_Down_Test (Self : in out Instance) is
   begin
      -- Ensure component resources are reclaimed:
      Self.Tester.Component_Instance.Destroy;
      -- Free tester heap:
      Self.Tester.Final_Base;
   end Tear_Down_Test;

   -------------------------------------------------------------------------
   -- Tests:
   -------------------------------------------------------------------------

   -- Validate motor step command output against reference Python scenarios.
   overriding procedure Test (Self : in out Instance) is
      T : Component.Stepper_Motor_Controller.Implementation.Tester.Instance_Access renames Self.Tester;

      use Parameter_Enums.Parameter_Update_Status;

      function To_Radians (Degrees : Long_Float) return Short_Float is
      begin
         return Short_Float (Degrees * Ada.Numerics.Pi / 180.0);
      end To_Radians;

      function Compute_Expected_Steps (
         Theta_Init : Short_Float;
         Theta_Ref  : Short_Float;
         Step_Angle : Short_Float;
         Theta_Min  : Short_Float;
         Theta_Max  : Short_Float
      ) return Integer_32 is
         Theta_Init_SF : constant Short_Float := Theta_Init;
         Theta_Ref_SF  : constant Short_Float := Theta_Ref;
         Step_Angle_SF : constant Short_Float := Step_Angle;
         Theta_Min_SF  : constant Short_Float := Theta_Min;
         Theta_Max_SF  : constant Short_Float := Theta_Max;

         function Round_Up_To_Step (Value : Short_Float; Step : Short_Float) return Short_Float is
            Quotient : constant Short_Float := Value / Step;
         begin
            return Short_Float (Long_Float'Ceiling (Long_Float (Quotient)) * Long_Float (Step));
         end Round_Up_To_Step;

         function Round_Down_To_Step (Value : Short_Float; Step : Short_Float) return Short_Float is
            Quotient : constant Short_Float := Value / Step;
         begin
            return Short_Float (Long_Float'Floor (Long_Float (Quotient)) * Long_Float (Step));
         end Round_Down_To_Step;

         Adjusted_Theta : Short_Float;
         Temp_Steps     : Short_Float;
         Steps_Long     : Long_Float;
         Lower_Fraction : Long_Float;
         Upper_Fraction : Long_Float;
      begin
         if Theta_Ref_SF >= Theta_Max_SF or else Theta_Ref_SF <= Theta_Min_SF then
            return 0;
         end if;

         pragma Assert (Step_Angle_SF > 0.0, "Step angle must be positive.");

         if Theta_Init_SF > 0.0 then
            Adjusted_Theta := Round_Up_To_Step (Theta_Init_SF, Step_Angle_SF);
         else
            Adjusted_Theta := Round_Down_To_Step (Theta_Init_SF, Step_Angle_SF);
         end if;

         Temp_Steps := (Theta_Ref_SF - Adjusted_Theta) / Step_Angle_SF;
         Steps_Long := Long_Float (Temp_Steps);
         Lower_Fraction := Steps_Long - Long_Float'Floor (Steps_Long);
         Upper_Fraction := Long_Float'Ceiling (Steps_Long) - Steps_Long;

         if Upper_Fraction > Lower_Fraction then
            Steps_Long := Long_Float'Floor (Steps_Long);
         else
            Steps_Long := Long_Float'Ceiling (Steps_Long);
         end if;

         return Integer_32 (Steps_Long);
      end Compute_Expected_Steps;

      function To_Sys_Time (Seconds : Long_Float) return Sys_Time.T is
         Whole_Seconds : Interfaces.Unsigned_32 := Interfaces.Unsigned_32 (Integer (Long_Float'Floor (Seconds)));
         Fraction : constant Long_Float := Seconds - Long_Float'Floor (Seconds);
         Sub_Int  : Integer := Integer (Long_Float'Floor (Fraction * 65536.0 + 0.5));
      begin
         if Sub_Int = 65536 then
            Whole_Seconds := Whole_Seconds + Interfaces.Unsigned_32 (1);
            Sub_Int := 0;
         end if;

         return (
            Seconds    => Whole_Seconds,
            Subseconds => Interfaces.Unsigned_16 (Sub_Int)
         );
      end To_Sys_Time;

      Two_Pi : constant Short_Float := Short_Float (2.0 * Ada.Numerics.Pi);
      Theta_Max_Default : constant Short_Float := Two_Pi;
      Theta_Min_Default : constant Short_Float := -Two_Pi;

      Step_Angles : constant array (Positive range <>) of Short_Float := [
         To_Radians (0.008),
         To_Radians (0.01),
         To_Radians (0.5)
      ];

      Step_Times : constant array (Positive range <>) of Short_Float := [
         0.008,
         0.1,
         0.5
      ];

      Theta_Inits : constant array (Positive range <>) of Short_Float := [
         To_Radians (-5.0),
         To_Radians (0.0),
         To_Radians (60.0)
      ];

      Theta_Refs : constant array (Positive range <>) of Short_Float := [
         To_Radians (0.0),
         To_Radians (10.6),
         To_Radians (60.0051)
      ];

      Interrupt_Theta_Refs_1 : constant array (Positive range <>) of Short_Float := [
         To_Radians (-10.0),
         To_Radians (10.0)
      ];

      Interrupt_Theta_Refs_2 : constant array (Positive range <>) of Short_Float := [
         To_Radians (0.0),
         To_Radians (5.0),
         To_Radians (10.0)
      ];

      Interrupt_Fractions : constant array (Positive range <>) of Short_Float := [
         0.0,
         0.25,
         0.5,
         0.75
      ];

      procedure Configure_Parameters (
         Step_Angle : Short_Float;
         Step_Time  : Short_Float;
         Theta_Min  : Short_Float;
         Theta_Max  : Short_Float
      ) is
         Params : Stepper_Motor_Controller_Parameters.Instance;
         Result : Parameter_Enums.Parameter_Update_Status.E;
      begin
         Result := T.Stage_Parameter (Stepper_Motor_Controller_Parameters.Theta_Max (Params, (Value => Theta_Max)));
         pragma Assert (Result = Parameter_Enums.Parameter_Update_Status.Success);
         Result := T.Stage_Parameter (Stepper_Motor_Controller_Parameters.Theta_Min (Params, (Value => Theta_Min)));
         pragma Assert (Result = Parameter_Enums.Parameter_Update_Status.Success);
         Result := T.Stage_Parameter (Stepper_Motor_Controller_Parameters.Step_Angle (Params, (Value => Step_Angle)));
         pragma Assert (Result = Parameter_Enums.Parameter_Update_Status.Success);
         Result := T.Stage_Parameter (Stepper_Motor_Controller_Parameters.Step_Time (Params, (Value => Step_Time)));
         pragma Assert (Result = Parameter_Enums.Parameter_Update_Status.Success);
         pragma Assert (T.Validate_Parameters = Parameter_Enums.Parameter_Update_Status.Success);
         pragma Assert (T.Update_Parameters = Parameter_Enums.Parameter_Update_Status.Success);
      end Configure_Parameters;

      procedure Run_Scenario (
         Theta_Init : Short_Float;
         Theta_Ref  : Short_Float;
         Step_Angle : Short_Float;
         Step_Time  : Short_Float;
         Theta_Min  : Short_Float := Theta_Min_Default;
         Theta_Max  : Short_Float := Theta_Max_Default;
         Expect_Output : Boolean := True
      ) is
         Steps_Expected : constant Integer_32 :=
           Compute_Expected_Steps (Theta_Init, Theta_Ref, Step_Angle, Theta_Min, Theta_Max);
         Message_Time_Sec : constant Long_Float := 1.0;
         Message_Time     : constant Sys_Time.T := To_Sys_Time (Message_Time_Sec);
      begin
         T.Motor_Step_Command_T_Recv_Sync_History.Clear;
         T.Data_Product_Fetch_T_Service_History.Clear;

         T.Data_Dependency_Return_Status_Override := Data_Product_Enums.Fetch_Status.Success;
         T.Data_Dependency_Return_Id_Override := 0;
         T.Data_Dependency_Return_Length_Override := 0;
         T.Data_Dependency_Timestamp_Override := (0, 0);

         T.System_Time := To_Sys_Time (0.0);
         T.Motor_Reference_Angle := (Theta => Theta_Init, Theta_Dot => 0.0);

         T.Component_Instance.Init;

         Configure_Parameters (
            Step_Angle => Step_Angle,
            Step_Time  => Step_Time,
            Theta_Min  => Theta_Min,
            Theta_Max  => Theta_Max
         );

         T.Component_Instance.Map_Data_Dependencies (
            Motor_Reference_Angle_Id => 0,
            Motor_Reference_Angle_Stale_Limit => Ada.Real_Time.Time_Span_Zero
         );

         T.Component_Instance.Set_Up;

         T.Motor_Reference_Angle := (Theta => Theta_Ref, Theta_Dot => 0.0);
         T.Data_Dependency_Timestamp_Override := Message_Time;
         T.System_Time := Message_Time;
         T.Tick_T_Send ((Time => Message_Time, Count => 0));

         if Expect_Output then
            Natural_Assert.Eq (T.Motor_Step_Command_T_Recv_Sync_History.Get_Count, 1);
            declare
               Output : constant Motor_Step_Command.T := T.Motor_Step_Command_T_Recv_Sync_History.Get (1);
            begin
               Integer_Assert.Eq (Integer (Output.Steps_Commanded), Integer (Steps_Expected));
            end;
         else
            Natural_Assert.Eq (T.Motor_Step_Command_T_Recv_Sync_History.Get_Count, 0);
         end if;

         T.Component_Instance.Destroy;
      end Run_Scenario;

      procedure Run_Interrupt_Scenario (
         Theta_Ref_1 : Short_Float;
         Theta_Ref_2 : Short_Float;
         Interrupt_Fraction : Short_Float
      ) is
         Step_Angle : constant Short_Float := To_Radians (1.0);
         Step_Time  : constant Short_Float := 1.0;
         Theta_Init : constant Short_Float := 0.0;
         Theta_Min  : constant Short_Float := Theta_Min_Default;
         Theta_Max  : constant Short_Float := Theta_Max_Default;

         Steps_Commanded_1 : constant Integer_32 :=
           Compute_Expected_Steps (Theta_Init, Theta_Ref_1, Step_Angle, Theta_Min, Theta_Max);

         Actuate_Time1 : constant Long_Float :=
           Long_Float (Step_Time) * Long_Float (abs (Integer (Steps_Commanded_1)));
         Sim_Time1 : constant Long_Float :=
           (Actuate_Time1 / 2.0) + (Long_Float (Interrupt_Fraction) * Long_Float (Step_Time));

         Message_Time1_Sec : constant Long_Float := 1.0;
         Message_Time1 : constant Sys_Time.T := To_Sys_Time (Message_Time1_Sec);
         Message_Time2 : constant Sys_Time.T := To_Sys_Time (Message_Time1_Sec + Sim_Time1);

         Interrupted_Motor_Angle : Long_Float := Long_Float (Theta_Init);
         Step_Angle_Long : constant Long_Float := Long_Float (Step_Angle);
         Step_Time_Long  : constant Long_Float := Long_Float (Step_Time);
      begin
         T.Motor_Step_Command_T_Recv_Sync_History.Clear;
         T.Data_Product_Fetch_T_Service_History.Clear;

         T.Data_Dependency_Return_Status_Override := Data_Product_Enums.Fetch_Status.Success;
         T.Data_Dependency_Return_Id_Override := 0;
         T.Data_Dependency_Return_Length_Override := 0;
         T.Data_Dependency_Timestamp_Override := (0, 0);

         T.System_Time := To_Sys_Time (0.0);
         T.Motor_Reference_Angle := (Theta => Theta_Init, Theta_Dot => 0.0);

         T.Component_Instance.Init;

         Configure_Parameters (
            Step_Angle => Step_Angle,
            Step_Time  => Step_Time,
            Theta_Min  => Theta_Min,
            Theta_Max  => Theta_Max
         );

         T.Component_Instance.Map_Data_Dependencies (
            Motor_Reference_Angle_Id => 0,
            Motor_Reference_Angle_Stale_Limit => Ada.Real_Time.Time_Span_Zero
         );

         T.Component_Instance.Set_Up;

         -- First command
         T.Motor_Reference_Angle := (Theta => Theta_Ref_1, Theta_Dot => 0.0);
         T.Data_Dependency_Timestamp_Override := Message_Time1;
         T.System_Time := Message_Time1;
         T.Tick_T_Send ((Time => Message_Time1, Count => 0));

         Natural_Assert.Eq (T.Motor_Step_Command_T_Recv_Sync_History.Get_Count, 1);
         declare
            Output_1 : constant Motor_Step_Command.T := T.Motor_Step_Command_T_Recv_Sync_History.Get (1);
         begin
            Integer_Assert.Eq (Integer (Output_1.Steps_Commanded), Integer (Steps_Commanded_1));
         end;

         -- Advance time while command executes
         T.Data_Dependency_Timestamp_Override := Message_Time1;
         T.System_Time := Message_Time2;
         T.Tick_T_Send ((Time => Message_Time2, Count => 0));

         Natural_Assert.Eq (T.Motor_Step_Command_T_Recv_Sync_History.Get_Count, 1);

         if Steps_Commanded_1 > 0 then
            Interrupted_Motor_Angle :=
              Long_Float (Theta_Init) + (Sim_Time1 / Step_Time_Long) * Step_Angle_Long;
            Interrupted_Motor_Angle :=
              Long_Float'Ceiling (Interrupted_Motor_Angle / Step_Angle_Long) * Step_Angle_Long;
         elsif Steps_Commanded_1 < 0 then
            Interrupted_Motor_Angle :=
              Long_Float (Theta_Init) - (Sim_Time1 / Step_Time_Long) * Step_Angle_Long;
            Interrupted_Motor_Angle :=
              Long_Float'Floor (Interrupted_Motor_Angle / Step_Angle_Long) * Step_Angle_Long;
         else
            Interrupted_Motor_Angle := Long_Float (Theta_Init);
         end if;

         declare
            Theta_Init_Second : constant Short_Float := Short_Float (Interrupted_Motor_Angle);
            Steps_Commanded_2 : constant Integer_32 :=
              Compute_Expected_Steps (Theta_Init_Second, Theta_Ref_2, Step_Angle, Theta_Min, Theta_Max);
         begin
            -- Second command arrives
            T.Motor_Reference_Angle := (Theta => Theta_Ref_2, Theta_Dot => 0.0);
            T.Data_Dependency_Timestamp_Override := Message_Time2;
            T.System_Time := Message_Time2;
            T.Tick_T_Send ((Time => Message_Time2, Count => 0));

            Natural_Assert.Eq (T.Motor_Step_Command_T_Recv_Sync_History.Get_Count, 2);
            declare
               Output_2 : constant Motor_Step_Command.T := T.Motor_Step_Command_T_Recv_Sync_History.Get (2);
            begin
               Integer_Assert.Eq (Integer (Output_2.Steps_Commanded), Integer (Steps_Commanded_2));
            end;
         end;

         T.Component_Instance.Destroy;
      end Run_Interrupt_Scenario;

   begin
      -- Nominal scenario sweep mirroring Python parameter set.
      for Theta_Init of Theta_Inits loop
         for Theta_Ref of Theta_Refs loop
            for Step_Angle of Step_Angles loop
               for Step_Time of Step_Times loop
                  Run_Scenario (
                     Theta_Init => Theta_Init,
                     Theta_Ref  => Theta_Ref,
                     Step_Angle => Step_Angle,
                     Step_Time  => Step_Time
                  );
               end loop;
            end loop;
         end loop;
      end loop;

      -- Out-of-bounds validation.
      Run_Scenario (
         Theta_Init => To_Radians (0.0),
         Theta_Ref  => To_Radians (275.0),
         Step_Angle => To_Radians (1.0),
         Step_Time  => 1.0,
         Theta_Min  => To_Radians (0.0),
         Theta_Max  => To_Radians (180.0)
      );

      -- Interrupt handling coverage.
      for Theta_Ref_1 of Interrupt_Theta_Refs_1 loop
         for Theta_Ref_2 of Interrupt_Theta_Refs_2 loop
            for Fraction of Interrupt_Fractions loop
               Run_Interrupt_Scenario (
                  Theta_Ref_1 => Theta_Ref_1,
                  Theta_Ref_2 => Theta_Ref_2,
                  Interrupt_Fraction => Fraction
               );
            end loop;
         end loop;
      end loop;
   end Test;

end Stepper_Motor_Controller_Tests.Implementation;
