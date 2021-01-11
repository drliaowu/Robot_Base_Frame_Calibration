% calibrate the LBR robot by move the joint one by one and find each joint
% axis

% circular movement one by one 
% the second, third, and fourth element give the x, y, and z position of a
% point
Point4Axis1 = [1 0.315882 0.0328608 -2.04561 -0.0103895 0.336546 0.150899 0.92944 -171.115 -21.5515 38.4962 
2 0.269763 0.0370386 -1.97508 -0.00215948 0.25365 0.165913 0.952958 -174.214 -21.2428 28.8632 
3 0.234996 0.0393285 -1.93657 0.00539424 0.192669 0.163417 0.967546 -176.752 -19.8049 21.9993 
4 0.187508 0.0411877 -1.8973 0.0197456 0.116833 0.160347 0.979923 179.927 -18.5775 13.6097 
5 0.143043 0.0432271 -1.86909 0.0235668 0.0543111 0.165309 0.984463 178.359 -18.9697 6.58783 
6 0.0895033 0.0450551 -1.84462 0.024407 0.0159239 0.189804 0.98139 177.599 -21.8434 2.32227 
7 0.038127 0.0463167 -1.82629 0.0318179 -0.064144 0.139714 0.9876 175.335 -16.3801 -6.76552 
8 -0.00751894 0.0469676 -1.81615 0.033768 -0.123244 0.126984 0.983639 174.231 -15.3966 -13.5255 
9 -0.0447107 0.0469564 -1.81182 0.0456087 -0.144029 0.171776 0.973483 171.756 -21.1273 -15.3522 
10 -0.0750507 0.0473433 -1.8107 0.0495765 -0.18565 0.170814 0.966385 170.244 -21.7712 -19.9916 
11 -0.127397 0.047422 -1.81515 0.0494818 -0.240779 0.143936 0.958572 169.422 -19.5839 -26.5746 
12 -0.173505 0.0464629 -1.82022 0.0473494 -0.29172 0.132664 0.946075 168.561 -19.3245 -32.6438 
13 -0.222981 0.0466557 -1.83398 0.0556253 -0.34505 0.127397 0.928233 165.803 -20.6497 -38.785 ];

Point4Axis1(:,2) = -Point4Axis1(:,2); %x axis of the measurement machine was wrongly defined

Point4Axis2 = [14 0.0738287 0.386424 -2.2076 0.00165272 -0.0188395 0.698433 0.715426 178.356 -88.643 -1.41236 
15 0.0756089 0.37669 -2.15703 0.0061429 -0.0174255 0.659447 0.751524 178.154 -82.5493 -1.03651 
16 0.0776548 0.356996 -2.09799 0.00980669 -0.0166562 0.601407 0.798709 177.954 -73.9728 -0.848652 
17 0.0789666 0.336455 -2.05407 0.0094237 -0.0193309 0.546943 0.836894 177.884 -66.3557 -1.26332 
18 0.0804166 0.312494 -2.01622 0.01383 -0.016156 0.510391 0.85968 177.692 -61.4109 -0.782716 
19 0.0814802 0.283134 -1.97879 0.0130586 -0.00999678 0.471369 0.881783 178.14 -56.2596 -0.304766 
20 0.0828758 0.246153 -1.94277 0.0173658 -0.0106956 0.412745 0.910618 177.681 -48.7716 -0.294722 
21 0.0837163 0.208422 -1.91229 0.0201722 -0.0126537 0.363277 0.931377 177.319 -42.6278 -0.510773 
22 0.0848207 0.171547 -1.89007 0.0213512 -0.0176298 0.305466 0.9518 177.052 -35.6168 -1.17555 
23 0.0848386 0.139009 -1.87306 0.022583 -0.00366738 0.289693 0.956846 177.401 -33.6803 0.347561 
24 0.0849326 0.112951 -1.8613 0.0229785 -0.00677962 0.234589 0.971799 177.258 -27.1461 -0.13727 
25 0.085178 0.0832119 -1.85173 0.0263638 -0.0118114 0.211263 0.977002 176.761 -24.4224 -0.684136 
26 0.0852401 0.042857 -1.83871 0.0276671 -0.0168104 0.143634 0.989101 176.584 -16.5683 -1.45011 
27 0.0854166 -0.0268161 -1.83184 0.0311515 -0.00942569 0.0800655 0.996258 176.354 -9.21469 -0.790278 
28 0.0855193 -0.0301857 -1.83172 0.0325841 -0.0100274 0.0851803 0.995782 176.181 -9.80603 -0.826183 
29 0.0853678 -0.0631594 -1.83271 0.032577 -0.0111317 0.0313924 0.998914 176.227 -3.63814 -1.1571 
30 0.0848248 -0.100967 -1.84064 0.029515 0.00551011 0.0164886 0.999413 176.628 -1.87017 0.686826 
31 0.0845879 -0.14191 -1.84748 0.0335336 0.0111744 -0.0214426 0.999145 176.129 2.49931 1.19709 
32 0.0835538 -0.1785 -1.8561 0.036194 -0.00828059 -0.110094 0.993228 175.981 12.6011 -1.39921 
33 0.0832384 -0.21299 -1.8707 0.0360117 0.00387139 -0.144767 0.988803 175.852 16.6528 -0.15874 
34 0.0822128 -0.25401 -1.89111 0.0383397 0.00285584 -0.210233 0.976895 175.635 24.2673 -0.603958 
35 0.0816038 -0.273923 -1.90376 0.0370825 -0.00163073 -0.237627 0.970647 175.915 27.47 -1.19123 ];

Point4Axis2(:,2) = -Point4Axis2(:,2);

Point4Axis3 = [36 0.31886 0.0332481 -2.05049 -0.0117708 0.34818 0.151641 0.925007 -170.452 -22.0864 39.8341 
37 0.294793 0.03569 -2.00942 -0.00214088 0.299983 0.154947 0.941274 -173.254 -20.782 34.3377 
38 0.266172 0.0376092 -1.97102 -0.00173266 0.247264 0.159229 0.955774 -174.659 -20.2579 28.1711 
39 0.248196 0.0386926 -1.95021 0.00511642 0.216236 0.152737 0.964307 -176.453 -18.779 24.7461 
40 0.220516 0.040247 -1.92354 0.00501692 0.176606 0.16566 0.970228 -177.022 -19.9079 20.1426 
41 0.195333 0.0408449 -1.90282 0.0158029 0.138403 0.17257 0.975097 -178.99 -20.2141 15.9839 
42 0.157048 0.0426048 -1.87784 0.0205723 0.0883346 0.156679 0.983476 179.255 -18.0359 10.3812 
43 0.125689 0.0439833 -1.8604 0.0210314 0.0465194 0.166001 0.984803 178.504 -19.062 5.65898 
44 0.0894363 0.0449139 -1.84434 0.0253619 0.0151198 0.189057 0.981522 177.472 -21.7555 2.25062 
45 0.0362579 0.0462565 -1.82627 0.0326098 -0.0585741 0.167152 0.983649 175.17 -19.5411 -5.98863 
46 0.00821234 0.0466822 -1.82011 0.0358542 -0.0883112 0.169607 0.980892 174.167 -20.0934 -9.26989 
47 -0.0308916 0.0471439 -1.8141 0.0386769 -0.142522 0.140902 0.978948 173.091 -17.3256 -15.5538 
48 -0.0802175 0.0475689 -1.81179 0.0431706 -0.192847 0.139036 0.970369 171.522 -17.9002 -21.2391 
49 -0.0927045 0.0476585 -1.81155 0.0476646 -0.212276 0.153434 0.963911 170.16 -20.1171 -23.2412 
50 -0.142448 0.0469203 -1.81434 0.0485278 -0.254455 0.141858 0.955392 169.215 -19.6071 -28.1928 
51 -0.183641 0.0471511 -1.824 0.0506399 -0.302704 0.127797 0.943119 167.975 -19.1132 -33.9196 ];

Point4Axis3(:,2) = -Point4Axis3(:,2);

Point4Axis4 = [52 0.0786098 -0.107166 -2.02613 0.0315124 0.0167467 -0.0307188 0.998891 176.33 3.5808 1.80628 
53 0.0801804 -0.0770204 -1.97485 0.0299747 0.00123516 0.0288549 0.999133 176.57 -3.30127 0.240529 
54 0.0814562 -0.0553941 -1.94365 0.0309964 -0.00752254 0.0503593 0.998222 176.408 -5.79751 -0.68163 
55 0.0832611 -0.0146093 -1.89594 0.0276756 0.0099504 0.120316 0.9923 176.988 -13.7869 1.5132 
56 0.0857268 0.0585691 -1.83091 0.02634 -0.00564419 0.179981 0.983301 176.914 -20.7474 -0.0927312 
57 0.0867515 0.0893347 -1.81037 0.025841 -0.0135691 0.204277 0.978478 176.783 -23.6103 -0.916585 
58 0.0877858 0.129246 -1.78709 0.0216533 -0.0193202 0.216784 0.975788 177.096 -25.0923 -1.62265 
59 0.0883446 0.162934 -1.77083 0.0223148 -0.0130171 0.271059 0.962216 177.134 -31.484 -0.74219 
60 0.0893914 0.198708 -1.75718 0.0247016 -0.0234223 0.305181 0.951686 176.483 -35.6114 -1.69072 
61 0.0893112 0.226074 -1.74753 0.0175017 -0.00501338 0.327093 0.944817 177.917 -38.1894 0.113211 
62 0.0897563 0.25677 -1.74042 0.0184436 -0.00784607 0.371424 0.928247 177.704 -43.6171 -0.0495842 
63 0.0902339 0.299357 -1.73028 0.0192912 -0.0172525 0.394094 0.918706 177.188 -46.4587 -0.945122 
64 0.0904449 0.340102 -1.72525 0.0146577 -0.01031 0.428867 0.90319 177.976 -50.8062 -0.346719 
65 0.0907761 0.399767 -1.72321 0.0123328 -0.016638 0.471252 0.881755 177.855 -56.2634 -1.01519 
66 0.0908538 0.448941 -1.72673 0.012071 -0.0180176 0.507554 0.861347 177.76 -61.039 -1.07639 
67 0.0909091 0.495861 -1.73392 0.013352 -0.018264 0.549544 0.835158 177.571 -66.71 -0.907119 
68 0.0905045 0.541293 -1.74512 0.0102798 -0.0168432 0.584411 0.811218 177.916 -71.5549 -0.877339 
69 0.0902537 0.583391 -1.75924 0.00734982 -0.0175997 0.619048 0.785121 178.09 -76.5275 -1.0621 
70 0.0898937 0.617486 -1.77333 0.00967557 -0.0200009 0.642658 0.765831 177.677 -80.0256 -1.04275 
71 0.0889445 0.677502 -1.8051 0.0039168 -0.0209506 0.685163 0.728078 178.027 -86.5462 -1.44057 
72 0.0881458 0.721606 -1.83476 0.00189618 -0.0225842 0.702431 0.711391 178.026 -89.3029 -1.68867 ];

Point4Axis4(:,2) = -Point4Axis4(:,2);

Point4Axis5 = [73 0.202393 0.697534 -1.87563 -0.819577 -0.0971685 0.0570183 0.561784 -68.2861 5.57953 -11.6912 
74 0.246384 0.648721 -1.87431 -0.772989 -0.0909528 0.0737087 0.623524 -77.4518 2.86613 -13.1425 
75 0.284253 0.58903 -1.87162 -0.711141 -0.0941155 0.0902574 0.69085 -88.2703 0.542715 -14.9758 
76 0.301598 0.55165 -1.87052 -0.67267 -0.0789259 0.0965096 0.729364 -94.8857 -2.04512 -14.18 
77 0.315873 0.508162 -1.86911 -0.6284 -0.0792506 0.102467 0.767029 -101.781 -3.41011 -14.4986 
78 0.325075 0.462775 -1.86669 -0.58046 -0.080202 0.115749 0.80202 -108.947 -5.50515 -15.2495 
79 0.328001 0.430578 -1.865 -0.549863 -0.0637284 0.110908 0.825402 -113.427 -6.66345 -13.1306 
80 0.328705 0.399483 -1.86412 -0.516285 -0.063107 0.117612 0.845952 -118.124 -7.90048 -13.1919 
81 0.32532 0.358623 -1.86165 -0.474121 -0.0577507 0.12063 0.870243 -123.832 -9.14337 -12.4097 
82 0.318975 0.321654 -1.85914 -0.429139 -0.0606749 0.134884 0.891047 -129.835 -11.1398 -12.938 
83 0.309122 0.286546 -1.85728 -0.389937 -0.049378 0.131543 0.910059 -134.764 -11.8145 -11.0964 
84 0.297829 0.255513 -1.85555 -0.348354 -0.0501563 0.135351 0.926182 -139.979 -12.689 -10.7899 
85 0.281968 0.222403 -1.85479 -0.306547 -0.0357597 0.153513 0.938714 -145.108 -15.6533 -9.27968 
86 0.267521 0.196771 -1.85315 -0.266505 -0.0364296 0.151717 0.95112 -149.895 -15.7996 -8.63648 
87 0.243974 0.164222 -1.85098 -0.219093 -0.0380545 0.143576 0.964332 -155.449 -15.2295 -7.83423 
88 0.21903 0.136252 -1.84843 -0.176014 -0.0341282 0.147272 0.97271 -160.445 -16.0473 -6.79041 
89 0.193921 0.112242 -1.84593 -0.132717 -0.0256245 0.15255 0.979009 -165.34 -17.0437 -5.20187 
90 0.169167 0.0925607 -1.84399 -0.0925284 -0.0283981 0.153452 0.983405 -169.986 -17.3144 -4.83296 
91 0.141731 0.0740622 -1.84329 -0.0522042 -0.0185435 0.157393 0.985981 -174.42 -17.9912 -3.03812 
92 0.101643 0.052416 -1.84296 0.00306646 0.00544075 0.183035 0.983087 179.769 -21.0922 0.677252 
93 0.075321 0.0413585 -1.84253 0.04018 -0.0053242 0.148526 0.988078 175.355 -17.0939 0.0810232 
94 0.0419309 0.0304663 -1.83834 0.0835231 0.00153317 0.143338 0.986142 170.54 -16.4125 1.54534 
95 0.00605934 0.0221921 -1.84105 0.129024 0.0147904 0.179224 0.975199 165.697 -20.2868 4.30671 
96 -0.0330612 0.0169639 -1.83579 0.178751 0.000464686 0.170148 0.969071 159.699 -19.283 3.53905 
97 -0.075842 0.0157803 -1.83841 0.230609 0.0248555 0.151322 0.960887 153.979 -16.3374 6.75125 
98 -0.129274 0.0208825 -1.83535 0.296487 0.0216806 0.163397 0.940705 146.219 -17.3011 7.91365 
99 -0.162 0.0275611 -1.83597 0.335642 0.0413128 0.173835 0.924889 141.82 -17.4251 11.1344 
100 -0.191261 0.0362499 -1.83527 0.370943 0.039765 0.174218 0.9113 137.458 -17.1021 11.6379 
101 -0.218847 0.0466385 -1.83366 0.406286 0.0325829 0.158711 0.899267 132.828 -15.2867 10.8107 
102 -0.246684 0.0596572 -1.83457 0.441077 0.0355804 0.155391 0.883198 128.382 -14.3653 11.5327 
103 -0.290407 0.085886 -1.83401 0.497514 0.0502809 0.152848 0.852402 120.975 -12.519 13.7572 
104 -0.322591 0.110726 -1.8344 -0.540538 -0.0518075 -0.155526 -0.825194 115.096 -11.9732 14.6929 
105 -0.338713 0.125029 -1.83567 -0.562294 -0.0670134 -0.162416 -0.808057 112.017 -11.2787 16.915 
106 -0.366515 0.154446 -1.83644 -0.602979 -0.0736076 -0.161877 -0.777685 105.994 -9.87157 18.0414 
107 -0.39199 0.187895 -1.83662 -0.644909 -0.0677419 -0.14654 -0.747014 99.5646 -7.90189 16.871 
108 -0.407548 0.212073 -1.83777 -0.674516 -0.0602911 -0.129371 -0.724332 94.9209 -6.31047 15.1809 
109 -0.430701 0.258722 -1.84009 -0.719197 -0.0709533 -0.134027 -0.678055 87.3323 -4.77534 16.7984 
110 -0.441887 0.289531 -1.84068 -0.745254 -0.0796362 -0.140105 -0.647013 82.532 -3.7778 18.1725 
111 -0.45435 0.338906 -1.84224 -0.786855 -0.0757065 -0.127963 -0.598961 74.8629 -2.0463 16.9818 
112 -0.460925 0.396615 -1.84587 -0.829639 -0.0878942 -0.117519 -0.538667 65.8192 1.15145 16.8393 
113 -0.460397 0.433077 -1.84714 -0.853944 -0.0985098 -0.1133 -0.498236 60.0284 3.31696 16.9577 
114 -0.451355 0.49844 -1.85011 -0.894888 -0.107851 -0.0966476 -0.422141 49.6162 6.63422 15.3096 
115 -0.451224 0.498274 -1.85025 -0.893519 -0.110881 -0.0997757 -0.423526 49.7797 6.78206 15.7967 
116 -0.438351 0.54211 -1.85337 -0.914573 -0.121082 -0.101131 -0.372382 43.0799 8.74437 15.9716 
117 -0.42718 0.570169 -1.85275 -0.932086 -0.10802 -0.0945632 -0.332572 38.2357 8.21803 14.3669 
118 -0.411518 0.600678 -1.85535 -0.946758 -0.102413 -0.0738374 -0.296157 33.8459 8.81812 11.5646 
119 -0.383855 0.644115 -1.85738 -0.964953 -0.102443 -0.0714738 -0.230787 25.9979 9.64915 10.6741 
120 -0.382922 0.644895 -1.85904 -0.962615 -0.116584 -0.0697371 -0.234343 26.2877 11.2612 10.8887 ];

Point4Axis5(:,2) = -Point4Axis5(:,2);

Point4Axis6 = [121 0.0979152 0.430572 -1.51292 -0.00709902 -0.0216649 0.78575 0.618124 178.551 -103.646 -2.17429 
122 0.098009 0.394336 -1.51272 -0.0017852 -0.024284 0.746719 0.664694 178.056 -96.686 -2.00283 
123 0.0980278 0.371564 -1.51428 0.00132517 -0.0231265 0.738416 0.673947 177.939 -95.2572 -1.67413 
124 0.097919 0.347825 -1.51873 -0.00234007 -0.0343025 0.696855 0.716388 177.449 -88.4832 -3.0042 
125 0.09773 0.318273 -1.52315 0.00648754 -0.0159739 0.70287 0.711109 178.184 -89.3447 -0.779161 
126 0.0970732 0.291919 -1.53131 0.00794296 -0.0204841 0.671308 0.740853 177.749 -84.3834 -1.12806 
127 0.0968389 0.269667 -1.53951 0.00597935 -0.0193859 0.644148 0.764632 178.044 -80.245 -1.25734 
128 0.0962318 0.243479 -1.55132 0.00887588 -0.0175129 0.612211 0.790451 177.967 -75.5332 -0.963665 
129 0.0955343 0.215894 -1.56712 0.00866644 -0.0146774 0.574999 0.817976 178.22 -70.2234 -0.804756 
130 0.0947274 0.185469 -1.58805 0.0111551 -0.0174114 0.53154 0.846781 177.856 -64.2534 -1.01009 
131 0.093691 0.155179 -1.61353 0.0146307 -0.0188584 0.479678 0.87712 177.492 -57.3706 -1.09132 
132 0.0927993 0.13154 -1.6397 0.0151326 -0.0136325 0.444301 0.895646 177.752 -52.7814 -0.628717 
133 0.09201 0.111572 -1.6657 0.0184596 -0.0108961 0.420962 0.906825 177.555 -49.808 -0.241792 
134 0.0915426 0.102306 -1.67798 0.0204512 -0.01092 0.385163 0.922557 177.355 -45.3264 -0.251793 
135 0.0905584 0.0871683 -1.70494 0.0191949 -0.00620255 0.364133 0.931129 177.692 -42.7148 0.139127 
136 0.0904692 0.0871831 -1.70502 0.0193622 -0.00636982 0.365459 0.930604 177.668 -42.8783 0.131587 
137 0.0897114 0.0726121 -1.73382 0.0193667 -0.0124505 0.303197 0.95265 177.452 -35.3244 -0.686307 
138 0.0886871 0.0656939 -1.75376 0.0274514 -0.00957768 0.306446 0.951444 176.669 -35.7083 -0.0802416 
139 0.0882039 0.0605073 -1.7672 0.0226099 -0.00489622 0.274859 0.961206 177.354 -31.912 0.172833 
140 0.0875399 0.0557868 -1.78502 0.0241173 -0.0138031 0.215842 0.976033 176.959 -24.9648 -0.947341 
141 0.0865313 0.0507803 -1.80489 0.0254246 -0.018728 0.193371 0.980617 176.725 -22.3545 -1.54128 
142 0.0864559 0.0502215 -1.80718 0.024535 -0.0224152 0.178284 0.983418 176.773 -20.6082 -2.02518 
143 0.0859688 0.0475691 -1.82448 0.0263618 -0.00748187 0.182616 0.982802 176.873 -21.0603 -0.290961 ];

Point4Axis6(:,2) = -Point4Axis6(:,2);

Point4Axis7 = [144 0.0855117 0.0454326 -1.83899 0.0272251 -0.0168612 0.145352 0.988862 176.631 -16.7668 -1.45732 
145 0.0653936 0.0482281 -1.77692 0.0482618 -0.22144 0.149126 0.962495 169.954 -19.7846 -24.3227 
146 0.0717021 0.047927 -1.79047 0.0395001 -0.180973 0.125256 0.974679 172.517 -15.9708 -20.0528 
147 0.0766841 0.0474211 -1.80265 0.0435686 -0.139577 0.170361 0.974473 172.121 -20.865 -14.903 
148 0.0791682 0.0469252 -1.80767 0.0368197 -0.113359 0.137054 0.983367 173.92 -16.5238 -12.2897 
149 0.0819947 0.0464537 -1.81998 0.0348626 -0.0775106 0.164859 0.982649 174.548 -19.4338 -8.09625 
150 0.0839854 0.0459605 -1.83065 0.0308281 -0.0496738 0.136149 0.988962 175.71 -15.8703 -5.15535 
151 0.0857836 0.0443962 -1.8591 0.0185437 0.0551005 0.199173 0.978238 179.173 -22.9688 6.61456 
152 0.0855198 0.0441183 -1.86508 0.0161159 0.077458 0.178464 0.980761 179.77 -20.6077 9.07271 
153 0.0850577 0.0438772 -1.86946 0.0137413 0.0903034 0.190005 0.977525 -179.566 -22.039 10.4728 
154 0.0840592 0.0435661 -1.87818 0.0145147 0.116062 0.139348 0.983311 -179.776 -16.1581 13.4323 
155 0.0809878 0.0427189 -1.89289 0.00559908 0.167636 0.17049 0.970979 -177.192 -20.3904 19.1142 
156 0.0783274 0.0422321 -1.90196 0.0032968 0.198848 0.159925 0.966888 -176.444 -19.4969 22.6799 
157 0.0742005 0.0416798 -1.91534 -0.000650443 0.235996 0.16092 0.958337 -175.035 -20.2511 26.8796 
158 0.063196 0.0400665 -1.93464 -0.00475439 0.304203 0.161508 0.938804 -172.506 -21.8688 34.725 
159 0.054992 0.0399282 -1.94677 -0.00988881 0.352071 0.147532 0.92422 -170.772 -21.539 40.3807 
160 0.0477671 0.0398595 -1.95771 -0.0132855 0.385479 0.154043 0.909671 -168.499 -23.9063 44.2047 
161 0.0480168 0.0398656 -1.95781 -0.013523 0.384772 0.152689 0.910195 -168.58 -23.6881 44.1312 
162 0.0164607 0.051712 -1.72611 0.0733968 -0.44993 0.125781 0.88111 157.46 -27.0476 -50.7517 ];

Point4Axis7(:,2) = -Point4Axis7(:,2);

[xi1, origin1]=FindTwist(Point4Axis1(:,2:4));
[xi2, origin2]=FindTwist(Point4Axis2(:,2:4)); 
[xi3, origin3]=FindTwist(Point4Axis3(:,2:4));
[xi4, origin4]=FindTwist(Point4Axis4(:,2:4)); xi4 = -xi4;
[xi5, origin5]=FindTwist(Point4Axis5(:,2:4)); 
[xi6, origin6]=FindTwist(Point4Axis6(:,2:4)); 
[xi7, origin7]=FindTwist(Point4Axis7(:,2:4)); xi7 = -xi7;
xi = [xi1,xi2,xi3,xi4,xi5,xi6,xi7];

Pose_st=[2 0.0850219 0.0457029 -1.8361 0.0281346 -0.0222064 0.13533 0.990152 176.459 -15.6299 -2.08377];
Pst = Pose_st(2:4)';
Qst = Pose_st(5:8)';
Rst = Q2R(Qst);
gst = [-1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1]*[Rst,Pst;0,0,0,1]*[-1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1];