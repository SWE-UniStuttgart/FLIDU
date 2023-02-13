# FLIDU
FLIDU (Floating Lidar Uncertainty) is a tool for the quantification of motion induced uncertainties of nacelle based lidar inflow measurments on floating offshore wind turbines (FOWT). The framework contains an analytical model of the wind field as well as the lidar measurements under consideration of FOWT dynamics. FOWT dynamics are modelled as harmonic oscillation and parameterized by their amplitude, frequency and mean value. 
Measurement uncertainty is derived by applying the GUM methodology on the equatuation of the analytical model. Detailed information on the underlying model and the derivation of uncertainty can be found in [1].

## Usage 

### FLIDU_v1_0.m 
Main script of the tool. It reads settings from UncertaintyConfig.m, calls the uncertainty and bias calculation functions and plots uncertainty and bias results for the defined parameters.

### UncertaintyConfig.m
In this function all parameters of the model, including lidara configuration, dynamics parameters and wind field parameters are specified. 

## Resources

[1] Gräfe M J, Pettas V, Gottschall J and Cheng P W 2023 Correction of motion influence for nacelle
based lidar systems on floating wind turbines Wind Energy Science Discussions 2023 1–39 URL
https://wes.copernicus.org/preprints/wes-2023-11/


