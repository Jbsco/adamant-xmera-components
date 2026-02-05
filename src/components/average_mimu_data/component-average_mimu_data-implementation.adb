--------------------------------------------------------------------------------
-- Average_Mimu_Data Component Implementation Body
--------------------------------------------------------------------------------

with Acc_Data.C;
with Imu_Sensor_Body.C;
with Packed_F32x9.C;

package body Component.Average_Mimu_Data.Implementation is

   --------------------------------------------------
   -- Subprogram for implementation init method:
   --------------------------------------------------
   -- Initializes the AverageMimuData algorithm.
   overriding procedure Init (Self : in out Instance) is
   begin
      -- Allocate C++ class on the heap
      Self.Alg := Create;
   end Init;

   not overriding procedure Destroy (Self : in out Instance) is
   begin
      -- Free the C++ heap data.
      Destroy (Self.Alg);
   end Destroy;

   ---------------------------------------
   -- Invokee connector primitives:
   ---------------------------------------
   -- Receive accelerometer buffer data for averaging.
   overriding procedure Acc_Data_T_Recv_Sync (Self : in out Instance; Arg : in Acc_Data.T) is
   begin
      -- Update the parameters:
      Self.Update_Parameters;

      declare
         -- Convert Ada types to C types:
         Acc_Data_C : aliased Acc_Data.C.U_C := Acc_Data.C.To_C (Acc_Data.Unpack (Arg));

         -- Call the C algorithm:
         Imu_Output : constant Imu_Sensor_Body.C.U_C := Update (
            Self.Alg,
            Acc_Data_In => Acc_Data_C'Unchecked_Access
         );
      begin
         -- Send out data product:
         Self.Data_Product_T_Send (Self.Data_Products.Imu_Body_Data (
            Self.Sys_Time_T_Get,
            Imu_Sensor_Body.Pack (Imu_Sensor_Body.C.To_Ada (Imu_Output))
         ));
      end;
   end Acc_Data_T_Recv_Sync;

   -- The parameter update connector.
   overriding procedure Parameter_Update_T_Modify (Self : in out Instance; Arg : in out Parameter_Update.T) is
   begin
      -- Process the parameter update, staging or fetching parameters as requested.
      Self.Process_Parameter_Update (Arg);
   end Parameter_Update_T_Modify;

   -----------------------------------------------
   -- Parameter handlers:
   -----------------------------------------------
   -- Description:
   --    Parameters for the Average Mimu Data component
   -- Apply parameter values to the C++ algorithm when parameters change.
   overriding procedure Update_Parameters_Action (Self : in out Instance) is
   begin
      -- Set the averaging window time delta:
      Set_Time_Delta (Self.Alg, Self.Time_Delta.Value);
      -- Set the platform-to-body DCM:
      Set_Dcm_Pltf_To_Bdy (Self.Alg, (Value => Packed_F32x9.C.To_C (Self.Dcm_Pltf_To_Bdy)));
   end Update_Parameters_Action;

   -- Invalid Parameter handler. This procedure is called when a parameter's type is found to be invalid:
   overriding procedure Invalid_Parameter (Self : in out Instance; Par : in Parameter.T; Errant_Field_Number : in Unsigned_32; Errant_Field : in Basic_Types.Poly_Type) is
      pragma Annotate (GNATSAS, Intentional, "subp always fails", "intentional assertion");
   begin
      -- None of the parameters should be invalid in this case.
      pragma Assert (False);
   end Invalid_Parameter;

end Component.Average_Mimu_Data.Implementation;
