## Field-Weakening Limitation

The golden FOC simulation includes field weakening above 600 RPM. The
field-weakening controller reduces the d-axis current reference (`Id_ref`)
when the commanded voltage approaches the available voltage limit.

For the 24 V base-voltage configuration, the controller is configured with:

- Field-weakening start speed: 600 RPM
- Minimum `Id_ref`: -14 A
- Voltage-margin target: 90%
- Current base: 20 A
- Voltage base: 24 V

The 0→500 RPM and 500→1000 RPM simulations converge to the commanded
speed. At the final 1000 RPM operating point, however, the field-weakening
controller reaches its configured `Id_ref` limit of -14 A.

Therefore, the 1000 RPM result demonstrates:

1. field-weakening activation,
2. negative d-axis current generation,
3. field-weakening current limiting, and
4. closed-loop convergence of the golden simulation.

It does NOT demonstrate that the requested 90% voltage-margin constraint is
physically satisfied at steady state at 1000 RPM.

The voltage-margin requirement is therefore treated as a documented
operating limitation of the Week-1 golden model rather than as a PASS
criterion for the 1000 RPM operating point.

This limitation is intentionally preserved for later design analysis and
must not be hidden or silently compensated for in the RTL implementation.