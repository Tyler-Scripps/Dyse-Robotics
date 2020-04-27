#include <math.h>

#define nStates 64
#define nActions 9

class QLearner{
	private:
		float discount;
		float impulse;
    float impulse_decay;
		int f_table[nActions][nStates];
	public:
		QLearner(float _discount, float _impulse, float i_decay){
			discount = _discount;
			impulse = _impulse;
			impulse_decay = i_decay;
		}
		
		void fill_p_table(float p_table[nActions][nStates]){
			for(int a = 0; a < nActions; a++){
				int total = 0;
				for(int i = 0; i < nStates; i++){
          total += f_table[a][i];
				}
				for(int ss = 0; ss < nStates; ss++){
					p_table[a][ss] = f_table[a][ss] / total;
				}
			}
		}
		
		void track_event(int s, int a, int ss, float p_table[nActions][nStates]){
			f_table[a][ss] += 1;
			impulse = impulse * impulse_decay;
      if( a >= 0)
			  fill_p_table(p_table);
		}
		
		void make_policy(float (*r)(int), float p_table[nActions][nStates], int policy[nStates]){
			float value_table[nStates][nActions] = {0.0};
			for(int i = 0; i < 20; i++){
				float prev_table[nStates][nActions];
				memcpy(prev_table, value_table, nStates * nActions * sizeof(float));
				for(int s = 0; s < nStates; s++){
					float max_q = 0;
					if(impulse < random(10)){
						for(int a = 0; a < nActions; a++){
							float q_value = 0;
							for(int ss = 0; ss < nStates; ss++){
								// the state_space must be visible for R(s) to be calculated
								q_value += p_table[a][ss] * (r(s) + discount * prev_table[s][a]);
							}
							if(q_value > max_q){
								max_q = q_value;
								policy[s] = a;
							}
							value_table[s][a] = max_q;
						}
					}
				}
			}
		}
};
