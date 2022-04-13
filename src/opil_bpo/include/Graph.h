#include "PQi.h"

class Edge 
{
public:

	int v, w;
	double wt;
    
	Edge(int v = -1, int w = -1, double wt = 0.0) : 
		v(v), 
		w(w), 
		wt(wt) { }
};

template <class Edge>
class DenseGRAPH
{ 
private:

	int Vcnt;
	int Ecnt;
	bool digraph;
	 vector <vector <Edge *> > adj;

public:

	DenseGRAPH(int V, bool digraph = false) :
		adj(V), 
		Vcnt(V), 
		Ecnt(0), 
		digraph(digraph)
	{ 
		for (int i = 0; i < V; i++)  adj[i].assign(V, 0);
	}

	int V() const { return Vcnt; }
	int E() const { return Ecnt; }
	bool directed() const { return digraph; }
	
	void insert(Edge *e)
	{ 
		int v = e->v;
		int w = e->w;
		if (adj[v][w] == 0) Ecnt++;		
		adj[v][w] = e;
		if (!digraph) adj[w][v] = e;
	} 

	void remove(Edge *e)
	{ 
		int v = e->v;
		int w = e->w;
		if (adj[v][w] != 0)  Ecnt--;
		adj[v][w] = 0;
		if (!digraph) adj[w][v] = 0; 
	} 

	Edge* edge(int v, int w) const 
	{ 
		return adj[v][w]; 
	}

	class adjIterator;
	friend class adjIterator;
};

template <class Edge>
class DenseGRAPH<Edge>::adjIterator
{ 
private:
    const DenseGRAPH &G;
    int i, v;

public:
    adjIterator(const DenseGRAPH<Edge> &G, int v) :
        G(G),
        v(v),
        i(0) {}
    Edge *beg() { i = -1; return nxt(); }
    Edge *nxt()
    {
        for (i++; i < G.V(); i++)
            if (G.edge(v, i))
                return G.adj[v][i];
        return 0;
    }
    bool end() const { return i >= G.V(); }
};
