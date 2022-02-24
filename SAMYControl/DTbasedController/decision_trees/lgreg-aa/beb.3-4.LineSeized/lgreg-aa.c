#include <stdio.h>

float classify(const float x[]);

int main() {
    float x[] = {2.f,0.f,0.f,2.f,0.f,0.f,2.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f,0.f};
    float result = classify(x);
    return 0;
}

float classify(const float x[]) {
	if (x[10] <= 1.5) {
		if (x[10] <= 0.5) {
			if (-3.4631688682488924*x[0]+0.3127407686914088*x[1]+0.12134478304713087*x[2]-3.4631688682488924*x[3]+0.3127407686936459*x[4]+0.12134478318712383*x[5]-3.4631688682488924*x[6]-0.5302910490211642*x[7]-32.7464293468772*x[8]+14.688046107572717*x[11]-1.5044195752887004*x[12]-1.5044195750284128*x[13]-5.510655798839544*x[14]-1.2983069296183005 <= 0) {
				if (x[2] <= 0.5) {
					if (-2.3955853930136475*x[0]+0.6974204142693428*x[1]-2.3955853930136475*x[3]+0.719545966253056*x[4]-0.2936717976042644*x[5]-2.3955853930136475*x[6]+0.7258360734220365*x[7]-0.37846980212573833*x[8]-0.36622281323585787*x[11]+9.556607661552162*x[12]-0.3417613774977343*x[13]-3.295879746884918*x[14]-1.7350567094094265 <= 0) {
						if (x[5] <= 0.5) {
							if (-2.3521803939884443*x[0]+1.1576751718914602*x[1]-2.3521803939884443*x[3]+1.1576751718914624*x[4]-2.3521803939884443*x[6]+1.187039660750147*x[7]+0.05532140714022815*x[8]+0.40815530549666507*x[11]+0.30665556998232524*x[12]+8.808560719800425*x[13]-3.344255084947914*x[14]-1.9513496045848435 <= 0) {
								if (-2.3475091445495173*x[0]+1.103875167389502*x[1]-2.3475091445495173*x[3]+1.1038751673895053*x[4]-2.3475091445495173*x[6]+1.4045477033850677*x[7]+3.193385239945057*x[8]+8.434139237618965*x[11]+0.03481646924046797*x[12]+0.03481646924046797*x[13]-2.3805321680866744*x[14]-2.24049369346635 <= 0) {
									if (-5.968594727861691*x[0]+18.56540653851435*x[1]-5.968594727861691*x[3]+18.565406538514356*x[4]-5.968594727861691*x[6]+18.56540653851435*x[7]+20.048097687458664*x[11]+20.04809768745864*x[12]+20.048097687458664*x[13]+2.2294408640217287*x[14]-19.047133592705812 <= 0) {
										if (1.8710421052487656*x[0]+1.8710421052487656*x[3]+1.8710421052487656*x[6]-17.059921663184664*x[11]+5.435190229536655*x[12]+5.435190229536655*x[13]-6.189541204111352*x[14]+0.9355210526243828 <= 0) {
											if (2.304705072538095*x[0]+2.304705072538095*x[3]+2.304705072538095*x[6]+1.1523525362690474*x[11]-13.662823723639656*x[12]+6.48939294561326*x[13]-6.021078241757349*x[14]+1.1523525362690474 <= 0) {
												return 2.0f;
											}
											else {
												return 1.0f;
											}

										}
										else {
											return 0.0f;
										}

									}
									else {
										return 3.0f;
									}

								}
								else {
									return 20.0f;
								}

							}
							else {
								return 12.0f;
							}

						}
						else {
							if (-0.7737213753426012*x[0]-0.22301777841190648*x[1]-0.7737213753426012*x[3]-0.2517031684308927*x[4]-0.20899100990396102*x[5]-0.7737213753426012*x[6]-0.10334351404222393*x[7]+2.3399968767096593*x[8]+6.003180451724622*x[11]-0.1934303438356503*x[12]-0.9671517191782538*x[13]-1.9360133865613642*x[14]-0.19343034383565036 <= 0) {
								return 17.0f;
							}
							else {
								return 15.0f;
							}

						}

					}
					else {
						return 11.0f;
					}

				}
				else {
					if (x[5] <= 0.5) {
						if (-0.8111636481388872*x[0]-0.31850062496964593*x[1]-0.21176484576250276*x[2]-0.8111636481388872*x[3]-0.25037296760517996*x[4]-0.8111636481388872*x[6]-0.2978548498599494*x[7]-0.10958074094277286*x[8]-0.5658590405119992*x[11]-1.013954560173607*x[12]+7.175137836995905*x[13]-2.1592969792077756*x[14]-0.2027909120347218 <= 0) {
							if (-0.7737213753428119*x[0]-0.2517031684310978*x[1]-0.20899100990365127*x[2]-0.7737213753428119*x[3]-0.22301777841213927*x[4]-0.7737213753428119*x[6]-0.10334351404255596*x[7]+2.3399968767105976*x[8]+6.003180451725611*x[11]-0.9671517191785163*x[12]-0.19343034383570298*x[13]-1.9360133865617357*x[14]-0.19343034383570346 <= 0) {
								return 18.0f;
							}
							else {
								return 16.0f;
							}

						}
						else {
							return 12.0f;
						}

					}
					else {
						if (-0.7301973056461584*x[0]-0.2496781863011086*x[1]-0.1994799374931932*x[2]-0.7301973056461584*x[3]-0.24967818630110797*x[4]-0.19947993749319312*x[5]-0.7301973056461584*x[6]-0.06988981605494177*x[7]+2.504000966402091*x[8]+6.593012658617431*x[11]-0.912746632057696*x[12]-0.912746632057696*x[13]-1.8764398226687806*x[14]-0.1825493264115396 <= 0) {
							return 14.0f;
						}
						else {
							return 13.0f;
						}

					}

				}

			}
			else {
				return 10.0f;
			}

		}
		else {
			if (-149.15862200260312*x[0]+9.27022299169355*x[1]-1.2235084561977823*x[2]-149.15862200260352*x[3]+9.270222991689872*x[4]-1.223508456197717*x[5]-149.15862200260239*x[6]+9.270222991691455*x[7]-1.223508456198423*x[8]-299.1054087145077*x[10]+315.3546116697933*x[11]+315.35461166979337*x[12]+315.3546116697945*x[13]-231.52391931074226*x[14]-2824.0665517326*x[15]-299.105408714506 <= 0) {
				if (x[11] <= 2.5) {
					if (-75.15831151606919*x[0]+3.8719049969755552*x[1]+6.3575020999976575*x[2]-75.15831151606871*x[3]+3.871904996975589*x[4]+6.357502099998654*x[5]+272.0375380357981*x[6]-95.64374378642276*x[7]-85.05210819517899*x[10]-170.10421639035798*x[11]-17.93947150871266*x[12]-17.939471508712707*x[13]+158.72776437846593*x[14]+101.05661472455374*x[15]-85.05210819517892 <= 0) {
						if (-9.51078513990945*x[0]+3.814589607309961*x[1]-0.11264712423480346*x[2]-9.51078513990945*x[3]+3.8145896073100327*x[4]-0.1126471242348012*x[5]-5.5268830803588065*x[6]+21.028887884894218*x[7]-3.113544858436145*x[10]-6.22708971687229*x[11]-2.3599055649814553*x[12]-2.3599055649814424*x[13]+32.42925556602931*x[14]+8.185690781622474*x[15]-3.1135448584361534 <= 0) {
							if (0.706489493814571*x[0]-2.721887341007087*x[1]+0.08435382704152684*x[2]+0.7064894938145742*x[3]-2.7218873410070796*x[4]+0.08435382704152708*x[5]-6.357939731338024*x[6]-4.225684160690001*x[7]+0.9263093881878134*x[10]+1.8526187763756268*x[11]+1.2795541350950983*x[12]+1.279554135095093*x[13]+5.957898030232441*x[14]+0.9263093881878101 <= 0) {
								return 19.0f;
							}
							else {
								return 4.0f;
							}

						}
						else {
							return 26.0f;
						}

					}
					else {
						return 23.0f;
					}

				}
				else {
					if (x[12] <= 2.5) {
						if (58.57429100918209*x[0]-48.34813639531108*x[1]-4.3838396767494645*x[3]+3.421293202607561*x[4]+0.4406441422820043*x[5]-48.130039486885416*x[6]+12.49488456272729*x[7]-0.6149843448914356*x[8]-12.032509871721354*x[10]-8.255685390942995*x[11]-24.065019743442708*x[12]-4.065307627532363*x[13]+82.43245541732547*x[14]+17.948978484144167*x[15]-12.032509871721322 <= 0) {
							if (9.320786863365246*x[0]-6.876349629356522*x[1]+5.129107848503029*x[3]+0.20704017084108503*x[4]+0.7141854535268674*x[5]+3.882924289317952*x[6]-0.1501207359153632*x[7]-0.3910735226117619*x[8]+0.970731072329488*x[10]-7.195942920882666*x[11]+1.941462144658976*x[12]+1.0189632950805576*x[13]-27.234164347915534*x[14]-5.649258732800785*x[15]+0.9707310723294857 <= 0) {
								if (-2.3562805440865127*x[0]-5.268407534057287*x[1]+0.9947079854710394*x[3]-2.7688538887349603*x[4]-0.2631703232972322*x[5]+2.311982290570019*x[6]-2.2639659099386718*x[7]-0.30424754336912574*x[8]+0.5779955726425048*x[10]+0.6272767353171238*x[11]+1.1559911452850096*x[12]+0.74139694080887*x[13]+1.1559911452850096*x[14]-0.586475961220916*x[15]+0.5779955726425057 <= 0) {
									return 27.0f;
								}
								else {
									return 5.0f;
								}

							}
							else {
								return 21.0f;
							}

						}
						else {
							return 24.0f;
						}

					}
					else {
						if (-27.942986462694662*x[0]+7.215086358463187*x[1]+0.5245844311632003*x[2]+55.03612903623332*x[3]-40.835971069241324*x[4]-27.942986462694662*x[6]+7.215086358463162*x[7]+0.5245844311627514*x[8]-6.985746615673666*x[10]-3.8160912342578923*x[11]-3.8160912342581845*x[12]-13.971493231347331*x[13]+66.2472131826041*x[14]+9.410629740200594*x[15]-6.985746615673666 <= 0) {
							if (7.435999786044772*x[0]-0.2607417466607291*x[1]-1.1089056994104205*x[2]+12.709028798946878*x[3]-9.061070489612563*x[4]+7.435999786044772*x[6]-0.26074174666071726*x[7]-1.1089056994104156*x[8]+1.858999946511193*x[10]-7.451682200403783*x[11]-7.45168220040378*x[12]+3.717999893022386*x[13]-31.014176193494876*x[14]-8.13649645316314*x[15]+1.8589999465111935 <= 0) {
								if (1.9874671866834608*x[0]-2.4602633082990844*x[1]-0.5048937915124406*x[2]-2.565782737550813*x[3]-5.339274496338798*x[4]+1.9874671866834608*x[6]-2.4602633082990866*x[7]-0.5048937915124428*x[8]+0.4968667966708652*x[10]+0.5777964053263059*x[11]+0.5777964053263102*x[12]+0.9937335933417304*x[13]+0.9937335933417304*x[14]-0.56175806882106*x[15]+0.496866796670867 <= 0) {
									return 28.0f;
								}
								else {
									return 6.0f;
								}

							}
							else {
								return 22.0f;
							}

						}
						else {
							return 25.0f;
						}

					}

				}

			}
			else {
				return 7.0f;
			}

		}

	}
	else {
		if (2.9534323963514564*x[0]+0.5681050049746804*x[1]-0.36067396193393714*x[2]+2.9534323963514564*x[3]+0.5681050049748723*x[4]-0.3606739619341287*x[5]+2.9534323963514564*x[6]+0.5681050049747954*x[7]-0.3606739619340504*x[8]-30.052431489978662*x[10]+2.9534323963514564*x[11]+2.9534323963514564*x[12]+2.9534323963514564*x[13]+19.08582666353598*x[14]+0.7383580990878811 <= 0) {
			return 9.0f;
		}
		else {
			return 8.0f;
		}

	}

}