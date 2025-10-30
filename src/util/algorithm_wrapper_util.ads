--------------------------------------------------------------------------------
-- Algorithm Wrapper Utility Package Spec
--------------------------------------------------------------------------------

with Data_Product_Enums;

-- Utility functions for algorithm wrapper components.
package Algorithm_Wrapper_Util is

   -- Check if a data dependency status indicates success.
   -- Returns True for Success, False for Not_Available or Stale, and asserts
   -- False for Error (which should never occur in normal operation).
   function Is_Dep_Status_Success (Status : in Data_Product_Enums.Data_Dependency_Status.E) return Boolean;

end Algorithm_Wrapper_Util;
