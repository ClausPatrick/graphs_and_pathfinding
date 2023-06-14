#include <iostream>
#include <cassert>
#include <vector>
#include <ctime>
#include "functions.h"

#define MAX_NODES 255
#define LOG_LEVEL 4

using std::string;
FILE* fptr;
time_t timer;
struct tm* tm_info;
char time_stamp[26];
char log_buffer[1024];

const char LOGFILE[] = "nodes_log.txt";
float Adj_Matrix[MAX_NODES][MAX_NODES];

int log(char* str, int level){
    static int index = 0;
    int return_val = 0;
    if (level<=LOG_LEVEL){
        const char* levels[] = {"CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG", "NOTSET"};
        timer = time(NULL);
        tm_info = localtime(&timer);

        strftime(time_stamp, 26, "%Y-%m-%d %H:%M:%S", tm_info);
        fptr = fopen(LOGFILE, "a");
        if (fptr==NULL){
            printf("File write error\n");
            return_val = -1;
        }else{
            fprintf(fptr, "%d [%s] %s: %s\n", index, time_stamp, levels[level], str);
            index++;
            //printf("write file: %s: %s\n", time_stamp, str);
        }
        fclose(fptr);
    }
    return return_val;
}

class Node {
    private:
        Node* Connected_Nodes[MAX_NODES];
        Node* Connected_Nodes_Sorted[MAX_NODES];
        int Connected_Node_Indices[MAX_NODES] = { 0 };
        int Indices_Sorted[MAX_NODES] = { 0 };
        int Indices_Sorting[MAX_NODES] = { 0 };
        static int Nodes_Counter;
        int Connection_Counter;

    public:
        int Index;
        int X;
        int Y;
        float Edge_Values[MAX_NODES] = { 0. };
        float Edge_Values_WLE[MAX_NODES] = { 0. };
        float Edge_Values_SLE[MAX_NODES] = { 0. };
        int Max_Edge = -1;
        int Min_Edge = -1;
        const char* Label;

//        Node(){
//            Index = Nodes_Counter;
//            X = -1;
//            Y = -1;
//            Label = "NO LABEL";
//
//            Connection_Counter = 0;
//            Nodes_Counter++;
//
//            sprintf(log_buffer, "Node %d, '%s', created at (%d, %d). Total %d Nodes.", Index, Label, X, Y, Nodes_Counter);
//            log(log_buffer, 4);
//        }

        Node(int x, int y, const char* label){
            Index = Nodes_Counter;
            X = x;
            Y = y;
            Label = label;


            Connection_Counter = 0;
            Nodes_Counter++;

            int i, j;
            for (i=0; i<MAX_NODES; i++){
                for (j=0; j<MAX_NODES; j++){
                    Adj_Matrix[i][j] = -1;
                }
            }
            sprintf(log_buffer, "Node %d, '%s', created at (%d, %d). Total %d Nodes.", Index, Label, X, Y, Nodes_Counter);
            log(log_buffer, 4);
        }


        int connect(Node* other_node, float edge_value){
            Connected_Nodes[Connection_Counter] = other_node;
            Edge_Values[Connection_Counter] = edge_value;
            Edge_Values_WLE[Connection_Counter] = edge_value;
            Edge_Values_SLE[Connection_Counter] = edge_value;
            Connected_Node_Indices[Connection_Counter] = other_node->Index;
            Indices_Sorting[Connection_Counter] = other_node->Index;
            int oi = Connected_Nodes[Connection_Counter]->Index;
            int ox = Connected_Nodes[Connection_Counter]->X;
            int oy = Connected_Nodes[Connection_Counter]->Y;
            //Connected_Nodes[Connection_Counter-1]->connect(this, edge_value);
            sprintf(log_buffer, "Node %d, '%s', at (%d, %d) to Node %d/'%s'/%d at (%d, %d), edge value: %2.2g. Nodes: %d.", Index, Label, X, Y, oi, other_node->Label,  Connected_Node_Indices[Connection_Counter], ox, oy, edge_value, Connection_Counter+1);
            log(log_buffer, 4);
            Connection_Counter++;
            Adj_Matrix[Index][other_node->Index] = edge_value;
            Adj_Matrix[Index][Index] = 0;
            return 0;
        }

        int bi_connect(Node* other_node, float edge_value){
            Connected_Nodes[Connection_Counter] = other_node;
            Edge_Values[Connection_Counter] = edge_value;
            Edge_Values_WLE[Connection_Counter] = edge_value;
            Edge_Values_SLE[Connection_Counter] = edge_value;
            int oi = Connected_Nodes[Connection_Counter]->Index;
            int ox = Connected_Nodes[Connection_Counter]->X;
            int oy = Connected_Nodes[Connection_Counter]->Y;
            Connected_Node_Indices[Connection_Counter] = oi;
            Connection_Counter++;
            Adj_Matrix[Index][other_node->Index] = edge_value;
            Adj_Matrix[Index][Index] = 0;

            Connected_Nodes[Connection_Counter-1]->connect(this, edge_value);
            sprintf(log_buffer, "Node %d, '%s', at (%d, %d) to Node %d/'%s'/%d at (%d, %d), edge value: %2.2g. Nodes: %d.", Index, Label, X, Y, oi, other_node->Label,  Connected_Node_Indices[Connection_Counter], ox, oy, edge_value, Connection_Counter+1);
            log(log_buffer, 4);
            return 0;
        }

        int map_route(){
            const float INFINITY = 9999;
            int i, j;
            float distance_matrix[Nodes_Counter][Nodes_Counter];
            float distance_vector[Nodes_Counter];
            float distance_vector_total[Nodes_Counter];
            int via_node[Nodes_Counter];

            bool nodes_visited[Nodes_Counter];
            int loop_counter, min_distance, next_index, prev_index;
            sprintf(log_buffer, "FB map_route()");
            log(log_buffer, 4);

            //----------------------------------------------------------------Dijkstra setup:
            for (i=0; i<Nodes_Counter; i++){
                distance_vector[i] = INFINITY;
                nodes_visited[i] = 0;

                for (j=0; j<Nodes_Counter; j++){
                    if (Adj_Matrix[i][j] == 0 || Adj_Matrix[i][j] == -1){
                        distance_matrix[i][j] = INFINITY;
                    }else{
                        distance_matrix[i][j] = Adj_Matrix[i][j];
                    }
                }
            }

            for (i=0; i<Nodes_Counter; i++){
                distance_vector[i] = distance_matrix[Index][i];
                //distance_vector_total[i] = distance_matrix[Index][i];
                distance_vector_total[i] = 0;
                via_node[i] = Index;

            }

            distance_vector[Index] = 0;
            distance_vector_total[Index] = 0;
            nodes_visited[Index] = 0;
            loop_counter = 1;
            next_index = Index;

            //----------------------------------------------------------------Dijkstra main loop:
            while (loop_counter < Nodes_Counter-1){
                min_distance = INFINITY;
                for (i=0; i<Nodes_Counter; i++){
                    sprintf(log_buffer, "DML (0): i: %2d, min dis: %4.1f, dis vec: %2.1f, is vis? %1d, loop c: %2d, dis>min? %1d.", i, min_distance, distance_vector[i], nodes_visited[i], loop_counter, (distance_vector[i] < min_distance));
                    log(log_buffer, 4);
                    if ((distance_vector[i] < min_distance) && (nodes_visited[i] == 0)){
                        min_distance = distance_vector[i];
                        prev_index = next_index;
                        next_index = i;
                        sprintf(log_buffer, "  DML (1): i: %d, min dis: %4.1f, dis vec: %4.1f, loop c: %d.", i, min_distance, distance_vector[i], loop_counter);
                        log(log_buffer, 4);
                    }
                }
                sprintf(log_buffer, "    DML (2): i: %d, next i: %d, min dis: %f, loop c: %d.", i, next_index, min_distance, loop_counter);
                log(log_buffer, 4);

                nodes_visited[next_index] = 1;
                for (i=0; i<Nodes_Counter; i++){
                    if (!nodes_visited[i]){
                        if ((min_distance + distance_matrix[next_index][i]) < distance_vector[i]){
                            float mpdm =  distance_vector[next_index] + min_distance + distance_matrix[next_index][i];
                            distance_vector[i] = min_distance + distance_matrix[next_index][i];
                            //distance_vector_total[i] = distance_vector_total[i] + min_distance + distance_matrix[next_index][i]  + distance_vector[next_index];
                            float dvni = distance_vector[next_index];
                            float dpdv = distance_vector[i] + distance_vector[next_index];
                            //distance_vector_total[i] = dpdv;
                            distance_vector_total[i] = distance_vector_total[next_index] + mpdm;
                            sprintf(log_buffer, "      DML (3): i: %d, mpdm: %2.2f, dpdv: %2.2f, next i: %d, via n: %d, dvni: %2.2f, loop c: %d.", i, mpdm, dpdv, next_index, via_node[i], dvni, loop_counter);
                            log(log_buffer, 4);
                            via_node[i] = next_index;
                        }
                    }
                }
                loop_counter++;
            }

            //----------------------------------------------------------------Dijkstra completed.

            for (i=0; i<Nodes_Counter; i++){
                for (j=0; j<Nodes_Counter; j++){
                    if (Adj_Matrix[i][j] == -1){
                        printf(".\t");
                    }else{
                        printf("%4.4g\t", Adj_Matrix[i][j]);
                    }
                }
                printf("\n");
            }
            for (i=0; i<Nodes_Counter; i++){
                 for (j=0; j<Nodes_Counter; j++){
                    if (distance_matrix[i][j] == -1){
                        printf(".\t");
                    }else{
                        printf("%4.4g\t", distance_matrix[i][j]);
                    }
                }
                printf("\n");

            }

            sprintf(log_buffer, "Path found for Node %d", Index);
            log(log_buffer, 4);

            sprintf(log_buffer, "  Distance / total:");
            log(log_buffer, 4);
            for (i=0; i<Nodes_Counter; i++){
                sprintf(log_buffer, "%2d:\t%2.2g\t%4.4g", i, distance_vector[i], distance_vector_total[i]);
                log(log_buffer, 4);
            }

            sprintf(log_buffer, "  via Node:");
            log(log_buffer, 4);
            for (i=0; i<Nodes_Counter; i++){
                sprintf(log_buffer, "%2d:\t%2d", i, via_node[i]);
                log(log_buffer, 4);
            }

            return 0;
        }

        void print_node(){
            int r, on;
            int min, max;
            sprintf(log_buffer, "Printing: Node %d at (%d, %d). Nodes: %d:", Index, X, Y, Connection_Counter);
            log(log_buffer, 4);
            printf("%s\n", log_buffer);
            if (Connection_Counter>0){
                for (on=0; on<Connection_Counter; on++){
                    int oi = Connected_Nodes[on]->Index;
                    int ox = Connected_Nodes[on]->X;
                    int oy = Connected_Nodes[on]->Y;
                    int onc = Connected_Nodes[on]->Connection_Counter;

                    sprintf(log_buffer, "\t%d: Node %d at (%d, %d) with value: %f. Nodes: %d:", on, oi, ox, oy, Edge_Values[on], onc);
                    log(log_buffer, 4);
                    printf("%s\n", log_buffer);
                }
                //int max_i = Connected_Nodes[max]->Index;
                //float max_e = Edge_Values[max];
                //int min_i = Connected_Nodes[min]->Index;
                //float min_e = Edge_Values[min];
                //sprintf(log_buffer, "\t\tWeakest   Node: %d. Value: %f:", min_i, min_e);
                //log(log_buffer, 4);
                //printf("%s\n", log_buffer);
                //sprintf(log_buffer, "\t\tStrongest Node: %d. Value: %f:", max_i, max_e);
                //log(log_buffer, 4);
                //printf("%s\n", log_buffer);
            }

        }
};

//
//class Graph {
//    private:
//        int Nodes_Counter = 0;
//    public:
//        Node Node_List[MAX_NODES];
//        Graph(){
//            int i;
//
//        }
//
//        int add_node(int x, int y, const char* label){
//            Node_List[Nodes_Counter] = Node(x, y, label);
//        }
//
//        void print_nodes(){
//            int i;
//            sprintf(log_buffer, "Printing Nodes: ");
//            log(log_buffer, 4);
//            for (i=0; i<Nodes_Counter; i++){
//                int index = Node_List[i].Index;
//                sprintf(log_buffer, "Index: %d", index);
//                log(log_buffer, 4);
//            }
//        }
//
//
//};
//
//


int Node::Nodes_Counter = 0;

int main(){
    int r;
//    Graph g = Graph();
//    g.add_node(9, 3, "Node_A");
//    g.add_node(0, 1, "Node_B");
//    g.add_node(2, 6, "Node_C");
//    g.add_node(5, 3, "Node_D");
//    g.print_nodes();


//    Node nA = Node(1, 2, "A");
//    Node nB = Node(2, 4, "B");
//    Node nC = Node(7, 2, "C");
//    Node nD = Node(9, 8, "D");
//    Node nE = Node(8, 9, "E");
//
//    r = nA.bi_connect(&nB, 6);
//    r = nA.bi_connect(&nD, 1);
//    r = nB.bi_connect(&nD, 2);
//    r = nB.bi_connect(&nC, 5);
//    r = nB.bi_connect(&nE, 2);
//    r = nC.bi_connect(&nE, 5);
//    r = nD.bi_connect(&nE, 1);
//
//
//    nA.map_route();

//1    Node n0 = Node(1, 2, "n0");
//1    Node n1 = Node(2, 4, "n1");
//1    Node n2 = Node(7, 2, "n2");
//1    Node n3 = Node(9, 8, "n3");
//1    Node n4 = Node(8, 9, "n4");
//1    Node n5 = Node(5, 8, "n5");
//1    Node n6 = Node(0, 7, "n6");
//1
//1    r = n0.bi_connect(&n1, 2);
//1    r = n0.bi_connect(&n2, 6);
//1    r = n1.bi_connect(&n3, 5);
//1    r = n2.bi_connect(&n3, 8);
//1    r = n3.bi_connect(&n5, 15);
//1    r = n3.bi_connect(&n4, 10);
//1    r = n4.bi_connect(&n6, 2);
//1    r = n5.bi_connect(&n6, 6);
//1    r = n5.bi_connect(&n4, 6);
//1
//1
//1    n0.map_route();

    Node n0 = Node(1, 2, "n0");
    Node n1 = Node(2, 4, "n1");
    Node n2 = Node(7, 2, "n2");
    Node n3 = Node(9, 8, "n3");
    Node n4 = Node(8, 9, "n4");
//    Node n5 = Node(5, 8, "n5");
//    Node n6 = Node(0, 7, "n6");
//    Node n7 = Node(3, 8, "n7");
//    Node n8 = Node(6, 6, "n8");
//    Node n9 = Node(0, 5, "n9");
//    Node n10 = Node(2, 8, "n10");
//
    r = n0.bi_connect(&n1, 1);
    r = n0.bi_connect(&n3, 2);
//    r = n0.bi_connect(&n6, 0.3);
//
    r = n1.bi_connect(&n2, 4);
//
//    r = n2.bi_connect(&n3, 0.5);
//    r = n2.bi_connect(&n8, 1.4);
//
    r = n3.bi_connect(&n4, 6);
//    r = n3.bi_connect(&n5, 0.7);
//    r = n3.bi_connect(&n7, 0.8);
//    r = n3.bi_connect(&n9, 1.5);
//
//    r = n5.bi_connect(&n9, 1.1);
//    r = n7.bi_connect(&n10, 0.9);
//    r = n8.bi_connect(&n10, 1.2);

    n0.map_route();

    sprintf(log_buffer, "Done.");
    log(log_buffer, 3);
    printf("Done.\n" );;

    return 0;
}
