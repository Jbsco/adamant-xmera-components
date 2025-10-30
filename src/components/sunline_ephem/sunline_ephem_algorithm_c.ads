pragma Ada_2012;

pragma Style_Checks (Off);
pragma Warnings (Off, "-gnatwu");

with Interfaces.C; use Interfaces; use Interfaces.C;
with Ephemeris.C;
with Nav_Att.C;
with Nav_Trans.C;

package Sunline_Ephem_Algorithm_C is

   -- MIT License
   -- *
   -- * Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
   -- * University of Colorado at Boulder
   -- *
   -- * Permission is hereby granted, free of charge, to any person
   -- * obtaining a copy of this software and associated documentation
   -- * files (the "Software"), to deal in the Software without restriction,
   -- * including without limitation the rights to use, copy, modify, merge,
   -- * publish, distribute, sublicense, and/or sell copies of the Software,
   -- * and to permit persons to whom the Software is furnished to do so,
   -- * subject to the following conditions:
   -- *
   -- * The above copyright notice and this permission notice shall be
   -- * included in all copies or substantial portions of the Software.
   -- *
   -- * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   -- * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   -- * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   -- * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
   -- * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   -- * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   -- * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   -- * SOFTWARE.
   --

   --* Opaque handle for a SunlineEphemAlgorithm instance.
   type Sunline_Ephem_Algorithm is limited private;
   type Sunline_Ephem_Algorithm_Access is access all Sunline_Ephem_Algorithm;

   --* @brief Construct a new SunlineEphemAlgorithm.
   function Create
     return Sunline_Ephem_Algorithm_Access
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineEphemAlgorithm_create";

   --* @brief Destroy a SunlineEphemAlgorithm.
   procedure Destroy
     (Self : Sunline_Ephem_Algorithm_Access)
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineEphemAlgorithm_destroy";

   --* @brief Compute ephemeris-based sunline heading in body frame.
   --* @param Self    The algorithm instance.
   --* @param Sun_Pos Pointer to sun ephemeris message payload.
   --* @param Sc_Pos  Pointer to spacecraft position message payload.
   --* @param Sc_Att  Pointer to spacecraft attitude message payload.
   --* @return Navigation message containing sunline direction in body frame.
   function Update
     (Self    : Sunline_Ephem_Algorithm_Access;
      Sun_Pos : Ephemeris.C.U_C_Access;
      Sc_Pos  : Nav_Trans.C.U_C_Access;
      Sc_Att  : Nav_Att.C.U_C_Access)
     return Nav_Att.C.U_C
     with Import       => True,
          Convention   => C,
          External_Name => "SunlineEphemAlgorithm_updateState";

private

   -- Private representation: opaque null record
   type Sunline_Ephem_Algorithm is null record;

end Sunline_Ephem_Algorithm_C;

pragma Style_Checks (On);
pragma Warnings (On, "-gnatwu");
