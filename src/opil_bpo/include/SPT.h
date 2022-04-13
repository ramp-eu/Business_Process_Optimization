#include "Graph.h"

template <class DenseGRAPH, class Edge> 
class SPT
{ 
//private:
public:

	const DenseGRAPH &G;
	vector<double> wt;
	vector<Edge *> spt;

//public:

	SPT(const DenseGRAPH &G, int s) : 
		G(G), 
		spt(G.V()), 
		wt(G.V(), 
		G.V())
	{ 
		PQi<double> pQ(G.V(), wt);
	
		for (int v = 0; v < G.V(); v++) pQ.insert(v);
	
		wt[s] = 0.0; 
		pQ.lower(s);  

		while (!pQ.empty()) 
		{ 
			int v = pQ.getmin(); 			
			if ( v != s && spt[v] == 0 ) {return;}
			typename DenseGRAPH::adjIterator A(G, v);       
			for ( Edge* e = A.beg(); !A.end(); e = A.nxt() ) 
			{ 
				int w = e->w; 
				double P = wt[v] + e->wt;
				
				if (P < wt[w]) 
				{ 
					//printf(" SPTclass:: Adding edge num=%d  <nw, nv, ewt> <%d, %d, %lf > Path_wt=%lf\n", w, e->w, e->v, e->wt, P);
					wt[w] = P; 
					pQ.lower(w); 
					spt[w] = e; 
				}
			}
		}
	}
	
	Edge *pathR(int v) const { return spt[v]; }
	double dist(int v) const { return wt[v]; }
};
