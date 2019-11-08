close all;
clear all;
clc;
freq_in = 122.88e6;
freq_out = 122.88e6;
vco_req_max = 6175e6;
vco_req_min = 5870e6;
PLL2_RDIV = (1:31);
PLL2_NDIV = (1:65535);
PLL2_PRESCALER = (3:6);
%% freq_out = (freq_in / PLL2_RDIV) * (VCO_OUT / PLL2_PRESCALER /PLL2_NDIV) / M
vco_expect_div = (ceil(vco_req_min/freq_out) : floor(vco_req_max/freq_out));
nn = 1;
for i = 1:length(vco_expect_div)
    vco_out_present = freq_out * vco_expect_div(i);
    for r = 1:length(PLL2_RDIV)
        f1 = freq_in / PLL2_RDIV(r);
        for n = 1:length(PLL2_NDIV)
            vco_out_actual = f1 * PLL2_NDIV(n);
            result = vco_out_present / vco_out_actual;
            for pre = 1:length(PLL2_PRESCALER)
                if result == PLL2_PRESCALER(pre)
                    result_R(nn) = PLL2_RDIV(r);
                    result_N(nn) = PLL2_NDIV(n);
                    result_RRE(nn) = PLL2_PRESCALER(pre);
                    nn = nn + 1;
                end
            end
        end
    end
end
result_cal = [result_R;result_N;result_RRE];
% X = [' result = ',num2str(result_cal)];
disp('Result(R:N:PRE) is below')
disp(result_cal);


