//
//  graph.h
//  adjacencylist
//
//  Created by Sam Goldman on 6/21/11.
//  Copyright 2011 Sam Goldman. All rights reserved.
//
//  Customized by Wu Jingbang on 2017
//
#include "graph.h"
#include "mbr-utils.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

using namespace mbr;

GeoHashRange lat_range, lon_range;

int compare_edges(const void *aa, const void *bb);
void vertex_free(void *data);

int compare_edges(const void *aa, const void *bb) {
    const MyEdge *a = (MyEdge*)aa;
    const MyEdge *b = (MyEdge*)bb;
    return (a->road_id > b->road_id) - (a->road_id < b->road_id);
}

void vertex_free(void *data) {
    Vertex *vertex = (Vertex*)data;
    list_free(vertex->edges);
    mbr_free(vertex);
}

MyGraph *MbrGraph::graph_create() {
    MyGraph *graph = (MyGraph*)mbr_malloc(sizeof(MyGraph));
    graph->vertices = list_create(vertex_free);
    return graph;
}

Vertex *MbrGraph::vertex_create(const char *idStr, double x, double y, uint64_t geoHash, int isIntersection) {
    Vertex *vertex = (Vertex*)mbr_malloc(sizeof(Vertex));
    strcpy(vertex->idStr, idStr);
    vertex->x = x;
    vertex->y = y;
    vertex->geoHash = geoHash;
    vertex->data = NULL;
    //vertex->edges = list_create(kfree);
    vertex->edges = list_create(mbr_free);
    vertex->indegree = 0;
    vertex->outdegree = 0;
    vertex->isIntersection = isIntersection;
    return vertex;
}

MyEdge *MbrGraph::edge_create(Vertex *vertex, int road_id) {
    MyEdge *edge = (MyEdge*)mbr_malloc(sizeof(MyEdge));
    //Edge *edge = (Edge*)malloc(sizeof(Edge));
    edge->vertex = vertex;
    edge->road_id = road_id;
    return edge;
}

void MbrGraph::graph_add_vertex(MyGraph *graph, Vertex *vertex) {
    list_add_data_tail(graph->vertices, vertex);
}

void MbrGraph::graph_add_vertex_sorted(MyGraph *graph, Vertex *vertex, int(*cmp)(const void *a, const void *b)) {
    list_add_data_sorted(graph->vertices, vertex, cmp);
}

void MbrGraph::graph_remove_vertex(MyGraph *graph, Vertex *vertex) {
    Node_list *n = graph->vertices->head;
    Node_list *prev_n = NULL;
    while (n) {
        if (n->data == vertex) {
            if (!prev_n) {
                graph->vertices->head = n->next;
            }
            else {
                prev_n->next = n->next;
            }
            graph->vertices->count--;
            mbr_free(n);
        }
        else {
            vertex_remove_edge_to_vertex((Vertex*)n->data, vertex);
        }
        prev_n = n;
        n = n->next;
    }
    vertex_free(vertex);
}

void MbrGraph::graph_remove_vertex_undirect(MyGraph *graph, Vertex *vertex) {
    Node_list *n = graph->vertices->head;
    Node_list *prev_n = NULL;
    while (n) {
        if (n->data == vertex) {
            if (!prev_n) {
                graph->vertices->head = n->next;
            }
            else {
                prev_n->next = n->next;
            }
            graph->vertices->count--;
            mbr_free(n);
        }
        else {
            vertex_remove_edge_to_vertex_undirect((Vertex*)n->data, vertex);
        }
        prev_n = n;
        n = n->next;
    }
    vertex_free(vertex);
}

void MbrGraph::vertex_add_edge(Vertex *vertex, MyEdge *edge) {
    list_add_data(vertex->edges, edge);
    edge->vertex->indegree++;
    vertex->outdegree++;
}

void MbrGraph::vertex_add_edge_sorted(Vertex *vertex, MyEdge *edge) {
    list_add_data_sorted(vertex->edges, edge, compare_edges);
    edge->vertex->indegree++;
    vertex->outdegree++;
}

void MbrGraph::vertex_remove_edge(Vertex *vertex, MyEdge *edge) {
    list_remove_data(vertex->edges, edge);
    edge->vertex->indegree--;
    vertex->outdegree--;
}

void MbrGraph::vertex_add_edge_to_vertex(Vertex *from, Vertex *to, int road_id) {
    MyEdge *edge = edge_create(to, road_id);
    list_add_data(from->edges, edge);
    to->indegree++;
    from->outdegree++;
}

void MbrGraph::vertex_add_edge_to_vertex_sorted(Vertex *from, Vertex *to, int road_id) {
    MyEdge *edge = edge_create(to, road_id);
    list_add_data_sorted(from->edges, edge, compare_edges);
    to->indegree++;
    from->outdegree++;
}

// here
void MbrGraph::vertex_remove_edge_to_vertex(Vertex *from, Vertex *to) {
    Node_list *e = from->edges->head;
    Node_list *prev_e = NULL;
    while (e) {
        if (((MyEdge *)e->data)->vertex == to) {
            if (!prev_e) {
                from->edges->head = e->next;
            }
            else {
                prev_e->next = e->next;
            }
            to->indegree--;
            from->outdegree--;
            //kfree(e->data);
            //kfree(e);
            mbr_free(e->data);
            mbr_free(e);
            break;
        }
        prev_e = e;
        e = e->next;
    }
}


void MbrGraph::vertex_add_edge_to_vertex_undirect(Vertex *from, Vertex *to, int road_id) {
	vertex_add_edge_to_vertex(from, to, road_id);
	vertex_add_edge_to_vertex(to, from, road_id);
}

void MbrGraph::vertex_add_edge_to_vertex_undirect_exclusive(Vertex *from, Vertex *to, int road_id){
	vertex_remove_edge_to_vertex_undirect(from, to);
	vertex_add_edge_to_vertex_undirect(from, to, road_id);
}
void MbrGraph::vertex_remove_edge_to_vertex_undirect(Vertex *from, Vertex *to) {
	vertex_remove_edge_to_vertex(from, to);
	vertex_remove_edge_to_vertex(to, from);
}

//void graph_sort_vertices(Graph *graph, int(*cmp)(const void *a, const void *b)) {
//    list_sort(graph->vertices, cmp);
//}
//
//void vertex_sort_edges(Vertex *vertex) {
//    list_sort(vertex->edges, compare_edges);
//}

int MbrGraph::graph_is_balanced(MyGraph *g) {
    Node_list *n = g->vertices->head;
//    Node_list *prev_n;
    while (n) {
        Vertex *v = (Vertex*)n->data;
        if (v->indegree != v->outdegree) {
            return 0;
        }
//        prev_n = n;
        n = n->next;
    }
    return 1;
}

void MbrGraph::graph_free(MyGraph *graph) {
    list_free(graph->vertices);
    //kfree(graph);
    mbr_free(graph);
}

Vertex * MbrGraph::getVertex(MyGraph *graph, const char* idStr) {
    Node_list *n = graph->vertices->head;
//    Node_list *prev_n = NULL;
    while (n)
    {
        if (!strcmp(((Vertex*)n->data)->idStr, idStr))
        {
            return (Vertex*)n->data;
        }
//        prev_n = n;
        n = n->next;
    }
    return NULL;
}

void MbrGraph::graph_print(MyGraph *g)
{
    Node_list *q;
    Node_list *p=g->vertices->head;
    while(p)
    {
        Vertex *v = (Vertex*)p->data;
        printf("%s %ld ",(char*)(v->idStr),v->geoHash);
        q = v->edges->head;
        while(q)
        {
            MyEdge *e = (MyEdge*)(q->data);
            printf("%s %d# ",(char*)(e->vertex->idStr), e->road_id);
            q=q->next;
        }
        printf("\n");
        p=p->next;
    }
}

//��������geoHash��ֵ�ľ���ֵ
uint64_t geohash_compare( uint64_t a, uint64_t b )
{
	if(a>b)
		return a-b;
	else
		return b-a;
}

#define LAT_STEP 0.0004 //45m
#define LON_STEP 0.0006 //51m
Vertex* find_vertex_by_square(MyGraph *g, double x, double y)
{
  Vertex *index,*vmin;
  double dist;
  double distmin = 999.0;
  double lat_range_min = y - LAT_STEP;
  double lat_range_max = y + LAT_STEP;
  double lon_range_min = x - LON_STEP;
  double lon_range_max = x + LON_STEP;

  Node_list *p=g->vertices->head;
  while(p)
    {
      index = (Vertex*)p->data;
      if (index->x > lon_range_min && index->x < lon_range_max
	  && index->y > lat_range_min && index->y < lat_range_max)
	{
	  dist = get_distance(index->y, index->x, y, x);
	  if (dist < distmin)
	    {
	      distmin = dist;
	      vmin = index;
	    }
	}
      p=p->next;
    }
  return vmin;
}
Vertex* MbrGraph::find_Vertex_by_VehiclePosition(MyGraph *g, uint64_t geoHash, double x, double y)
{
	uint64_t temp;
	double dist;
	double distmin = 999.0;
	uint64_t min1 = UINT64_MAX;
	uint64_t min2 = UINT64_MAX;
	uint64_t min3 = UINT64_MAX;
	Vertex *vmin, *v[3],*index;
	vmin = NULL;
	v[0] = NULL;
	v[1] = NULL;
	v[2] = NULL;
	Node_list *p=g->vertices->head;
	int i;
	while(p)//����һ��ͼ�ڵ㣬�ҵ������geoHash��ӽ���ͼ�ڵ㣻
	{
		index=(Vertex*)p->data;
		temp=geohash_compare(index->geoHash, geoHash);
		if(temp<min1)
		{
			min3 = min2;
			v[2] = v[1];
			min2 = min1;
			v[1] = v[0];
			min1=temp;
			v[0]=index;
		} else if (temp < min2) {
			min3 = min2;
			v[2] = v[1];
			min2 = temp;
			v[1] = index;
		} else if (temp < min3) {
			min3 = temp;
			v[2] = index;
		}
		p=p->next;
	}
	for (i=0; i<3 && v[i] != NULL; i++)
	{
		dist = get_distance(v[i]->y, v[i]->x, y, x);
		if (dist < distmin)
		{
			distmin = dist;
			vmin = v[i];
		}
	}

	//Check if it is a mutation point.
	if (distmin > 100)
	  {
	    vmin = find_vertex_by_square(g, x, y);
	  }
	return vmin;
}

typedef struct path          	//BFS�������ݽṹ��
{
	Vertex *v;					//���浱ǰ�ҵ��ĵ�·�ڵ�ָ�룻
	struct path *ancest;		//����BFS�������ϵĸ��ڵ㣻
	MyEdge *e;					//ָ�򸸽ڵ�ıߣ�
	struct path *next;
}path;

int find_vertex(path *head,path *index,Vertex *v)   //�ж�ͼ�ڵ��Ƿ��Ѿ�����������
{
    int flag = 0;
    path *temp=head;
    while(temp)
    {  
        if(temp == index)
            flag = 1;
        if(temp->v == v)
        {
            if(flag)
                temp->ancest = index;
            return 1;
        }
        temp=temp->next;
    }
    return 0;
}

Vertex* print_crossnode(path *p)    //����BFS����·���ϵĽ���ڵ���Ϣ��
{
    int first,second,count = 0;
    Vertex *ver = NULL;
    if(p != NULL && p->ancest != NULL)
    {
        second=p->e->road_id;
        p=p->ancest;
    }
    else
    {
        return NULL;
    }
    while(p->ancest != NULL)
    {
        first=second;
        second=p->e->road_id;
        if(first != second)
        {
            if(ver == NULL)
                ver = p->v;
            ++count;
        }
        p=p->ancest;
    }
    if(count == 1)
        return ver;
    else
    {
       return NULL;
    }
}


int free_path(path *p)
{
    path *temp;
    while(p)
    {
        temp=p;
        p=p->next;
        mbr_free(temp);
    }
    return 0;
}

int find_vertexlist(vertexlist *head, Vertex* v)
{
    vertexlist *temp = head;
    while(temp!=NULL)
    {
        if(temp->v == v)
            return 1;
        temp = temp->next;
    }
    return 0;
}

/**
 * set size of the intersection based on the width of the roads.
 * unimplemented yet!
 */
void MbrGraph::setIntersectionSize(GeoHashSetCoordinate * geohashset, Vertex * this_vertex, Vertex * dst_vertex)
{
    int i;
    geohashset->sx = 3;
    geohashset->sy = 3;
    geohashset->geohashset = (uint64_t**)mbr_malloc(sizeof(uint64_t*) * geohashset->sy);
    for (i = 0; i < geohashset->sy; ++i){
        geohashset->geohashset[i] = (uint64_t*)mbr_malloc(sizeof(uint64_t) * geohashset->sx);
    }
}

vertexlist* MbrGraph::cross_vertex(Vertex *from, Vertex *to)    //���Ҵ�from��to·���ϵĽ���·�ڽڵ㣻
{
    path *head,*tail,*index,*temp,*target = NULL;
    Vertex *v;
    vertexlist *head_list = NULL, *tail_list = NULL, *lst;
    int flag = 0;
    //��ʼ�����ݽṹ��
    head=tail=index=(path*)mbr_malloc(sizeof(path));
    if(!index)
    {
        //exit(0);
    }
    index->v=from;
    index->ancest=NULL;
    index->e=NULL;
    index->next=NULL;
    while(index)       //BFS������
    {
        if(strcmp(index->v->idStr,to->idStr)==0)
            break;
        Node_list *n=index->v->edges->head;
        while(n)
        {
            int ret = find_vertex(head,index,((MyEdge*)n->data)->vertex);

            if(ret)
            {
                if(target!=NULL&&strcmp(((MyEdge*)n->data)->vertex->idStr,to->idStr)==0)
                    flag = 1;
            }
            else
            {
                temp=(path*)mbr_malloc(sizeof(path));
                temp->v=((MyEdge*)n->data)->vertex;
                temp->ancest=index;
                temp->e=(MyEdge*)n->data;
                temp->next=NULL;

                //���±������Ľڵ���뵽����β����
                tail->next=temp;
                tail=temp;
                if(strcmp(temp->v->idStr,to->idStr)==0)
                {
                    flag = 1;
                    target = temp;
                }
            }
            if(flag == 1)   //�Ѿ�������Ŀ��ڵ㣬��ֹ������
            {
                v=print_crossnode(target);
                if(v != NULL && find_vertexlist(head_list,v) == 0)
                {

                    lst = (vertexlist*)mbr_malloc(sizeof(vertexlist));
                    lst->v =v;
                    lst->next = NULL;

                    if(head_list == NULL)
                    {
                        head_list = lst;
                        tail_list = lst;
                    }
                    else
                    {
                        tail_list->next = lst;
                        tail_list = lst;
                    }
                }
            }
            n=n->next;
        }
        index=index->next;
    }
    free_path(head);
    return head_list;
}


void make_coordinate_string(char* str, double x, double y)
{
    sprintf(str, "%09.6lf,%09.6lf", x, y);
    return;
}

void coordString_to_coordinate(const char* str, double &x, double &y)
{
    sscanf(str, "%lf,%lf", &x, &y);
    return;
}

int find_road_id(Vertex *from, Vertex *to)
{
    Node_list *e = from->edges->head;
    while (e) {
        if (((MyEdge *)e->data)->vertex == to) {
            return ((MyEdge *)e->data)->road_id;
        }
        e = e->next;
    }
    return -1;
}


void MbrGraph::edge_division(MyGraph *graph, Vertex* from, Vertex* to)
{
    int a;
    int road_id;
    double dst,lat1,lng1,lat2,lng2,lat3,lng3;
    GeoHashBits geo;
    Vertex *temp;
    char idStr[25];
    lat1 = from->y;
    lng1 = from->x;
    lat2 = to->y;
    lng2 = to->x;
    dst = get_distance(lat1, lng1, lat2, lng2);
    if(dst > MAX_DIST)
    {
        a = round(dst/AVERAGE_DIST);
        lat3 = lat1 + (lat2-lat1)/a;
        lng3 = lng1 + (lng2-lng1)/a;
        make_coordinate_string((char *)idStr, lat3, lng3);
        geohash_fast_encode(lat_range, lon_range, lat3, lng3, GEOHASH_STEP_BIT, &geo);
        temp = vertex_create(idStr, lng3, lat3, geo.bits, 0);
        graph_add_vertex(graph,temp);
        road_id = find_road_id(from,to);

        //printf("%d %d\n",dst,road_id);

        vertex_add_edge_to_vertex_undirect(from,temp,road_id);
        vertex_add_edge_to_vertex_undirect(temp,to,road_id);
        vertex_remove_edge_to_vertex_undirect(from,to);
    }
    return;
}

void MbrGraph::graph_division(MyGraph *graph)
{
    Node_list *n1;
    Node_list *n2=graph->vertices->head;

    lat_range.min = LAT_RANGE_MIN;
    lat_range.max = LAT_RANGE_MAX;
    lon_range.min = LON_RANGE_MIN;
    lon_range.max = LON_RANGE_MAX;

    while(n2)
    {
        Vertex *v = (Vertex*)n2->data;
        n1 = v->edges->head;
        while(n1)
        {
            MyEdge *e =(MyEdge*)(n1->data);
            n1 = n1->next;
            edge_division(graph,v,e->vertex);
            //graph_print(graph);
            //printf("####\n");
        }
        n2=n2->next;
    }
    return;
}
