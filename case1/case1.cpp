// Copyright (c) 2021 xxx.Co.Ltd. All rights reserved.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <algorithm>
#include <functional>
#include <utility>

//定义树结构下每个区域的结构体，用于生成最小生成树
struct treenode {
	int val; //区域编号
	int distance_to_other_point; //与不是该区域的出发区域的两个起点区域的距离和
	std::vector<treenode*> next; //存储叶子节点信息
	treenode() :val(-1) {}
	explicit treenode(int x) :val(x) {}
};
//定义区域的栅格信息，用于寻找最短路径
class grid_in_Dijkstra {
	public:
		int index;//区域编号
		int distance_to_ori;//与起始区域的距离
		int pre_index; //与起始区域连接路径上前向区域
		grid_in_Dijkstra() :index(-1), distance_to_ori(1e9), pre_index(-1) {}
		bool operator<(const grid_in_Dijkstra& a) const { //栅格比较函数
			return distance_to_ori > a.distance_to_ori; //大顶堆
		}
};
/*
	函数作用：输入参数读入，读取失败则直接退出
	N：区域总个数
	adjacency_matrix：图的邻接矩阵
	k_input_filename：输入文件名
*/
bool part_params_load(int &N, std::vector<std::vector<int> >&adjacency_matrix, const std::string k_input_filename) {
	std::ifstream myfile;
	myfile.open(k_input_filename);
	if (!myfile) {
		return 0;
	}
	myfile >> N;
	adjacency_matrix.resize(N);
	std::string s;
	getline(myfile, s);
	for (int i = 0; i < N; i++) {
		adjacency_matrix[i].resize(N);
		getline(myfile, s);
		int index = 0;
		for (int k = 0; k < s.size(); k++) {
			if (k == s.size() - 1) {
				int tmp = stoi(s.substr(index, k - index + 1));
				adjacency_matrix[i][tmp] = 1;
			} else if (s[k] == ' ') {
				int tmp = stoi(s.substr(index, k - index));
				adjacency_matrix[i][tmp] = 1;
				index = k + 1;
			}
		}
	}
	myfile.close();
	return 1;
}
/*
	函数作用：计算每个区域与起始区域0,1,2的距离
	N：区域总个数
	adjacency_matrix_except_endnode：除去终点的图的邻接矩阵
	dist：保存距离信息
*/
void calu_dist(int& N, const std::vector<std::vector<int> >&adjacency_matrix_except_endnode, std::vector<std::vector<int> >& dist) {
	for (int index = 0; index < 3; index++) {
		dist[index][index] = 0;
		std::vector<int> visit(N - 1, 0);
		std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int> >, std::greater<std::pair<int, int> > > pq;
		pq.push({ 0, index });
		while (!pq.empty()) {
			auto tmp = pq.top();
			pq.pop();
			int point_id = tmp.second;
			if (visit[point_id]) continue;
			visit[point_id] = 1;
			for (int i = 0; i < N - 1; i++) {
				if (adjacency_matrix_except_endnode[point_id][i] != 0 && visit[i] == 0) {
					dist[index][i] = std::min(dist[index][point_id] + 1, dist[index][i]);
					pq.push({ dist[index][i], i });
				}
			}
		}
	}
	return;
}
/*
	函数作用：根据与起始区域0,1,2的距离，初步分配每架飞行器的搜寻区域
	first_distri：初步分配的结果
	min_num_id：分配到的搜寻区域最小的飞行器的编号；
	mid_num_id：分配到的搜寻区域中等的飞行器的编号；
	max_num_id：分配到的搜寻区域最大的飞行器的编号；
	min_num，mid_num，max_num：搜寻区域数量；
*/
void first_distribution(int& N, const std::vector<std::vector<int> >& dist, std::vector<std::vector<std::vector<int> > >&first_distri,
	int& min_num_id, int& mid_num_id, int& max_num_id, int& min_num, int& mid_num, int& max_num) {
	for (int i = 0; i < N - 1; i++) {
		int flag = -1;
		int value = 1e9;
		for (int j = 0; j < 3; j++) {
			if (dist[j][i] <= value) {
				value = dist[j][i];
				flag = j;
			}
		}
		first_distri[flag].push_back({ value, i });
	}

	for (int i = 0; i < 3; i++) {
		if (first_distri[i].size() > max_num) {
			max_num = first_distri[i].size();
			max_num_id = i;
		}
		if (first_distri[i].size() < min_num) {
			min_num = first_distri[i].size();
			min_num_id = i;
		}
	}
	mid_num_id = 3 - min_num_id - max_num_id;
	mid_num = first_distri[mid_num_id].size();
}
/*
	函数作用：根据初步分配的结果，根据距离信息调整分配结果
	sub_arr：保留每个飞行器重分配后仍根据初分配保留下来的区域；
	extra_add：保留每个飞行器重分配后新增加的区域；
*/
void distribution_try(int add_min, int add_mid, const std::vector<std::vector<int> >& dist,
	int min_num_id, int mid_num_id, int max_num_id, int min_num, int mid_num, int max_num,
	std::vector<std::vector<std::vector<int> > >first_distri, std::vector<std::vector<int> >& sub_arr, std::vector<std::vector<int> >& extra_add) {

	for (int i = 0; i < first_distri[max_num_id].size(); i++) {
		first_distri[max_num_id][i].push_back(dist[min_num_id][first_distri[max_num_id][i][1]]);
	}
	for (int i = 0; i < first_distri[mid_num_id].size(); i++) {
		first_distri[mid_num_id][i].push_back(dist[min_num_id][first_distri[mid_num_id][i][1]]);
	}
	sort(first_distri[max_num_id].begin(), first_distri[max_num_id].end(), [=](std::vector<int>a, std::vector<int>b)->bool {
		if (a[0] == b[0]) {
			return a[2] > b[2];
		}
		return a[0] < b[0]; });
	sort(first_distri[mid_num_id].begin(), first_distri[mid_num_id].end(), [=](std::vector<int>a, std::vector<int>b)->bool {
		if (a[0] == b[0]) {
			return a[2] > b[2];
		}
		return a[0] < b[0]; });
	std::vector<std::vector<int> >undetermined_point;
	if (add_mid >= 0) {
		for (int i = 0; i < add_min + add_mid; i++) {
			int tmp = first_distri[max_num_id].back()[1];
			first_distri[max_num_id].pop_back();
			undetermined_point.push_back({ dist[min_num_id][tmp], tmp });
		}
		sort(undetermined_point.begin(), undetermined_point.end(), [=](std::vector<int>a, std::vector<int>b)->bool {return a[0] < b[0]; });
		for (int i = 0; i < add_min; i++) {
			extra_add[min_num_id].push_back(undetermined_point[i][1]);
		}
		for (int i = add_min; i < add_min + add_mid; i++) {
			undetermined_point[i][0] = dist[mid_num_id][undetermined_point[i][1]];
		}
		sort(undetermined_point.begin() + add_min, undetermined_point.begin() + add_min + add_mid,
			[=](std::vector<int>a, std::vector<int>b)->bool {return a[0] < b[0]; });
		for (int i = add_min; i < add_min + add_mid; i++) {
			extra_add[mid_num_id].push_back(undetermined_point[i][1]);
		}
	} else {
		for (int i = 0; i < add_min + add_mid; i++) {
			int tmp = first_distri[max_num_id].back()[1];
			first_distri[max_num_id].pop_back();
			undetermined_point.push_back({ dist[min_num_id][tmp], tmp });
		}
		for (int i = 0; i < (0 - add_mid); i++) {
			int tmp = first_distri[mid_num_id].back()[1];
			first_distri[mid_num_id].pop_back();
			undetermined_point.push_back({ dist[min_num_id][tmp], tmp });
		}
		sort(undetermined_point.begin(), undetermined_point.end(),
			[=](std::vector<int>a, std::vector<int>b)->bool {return a[0] < b[0]; });
		for (int i = 0; i < add_min - add_mid; i++) {
			extra_add[min_num_id].push_back(undetermined_point[i][1]);
		}
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < first_distri[i].size(); j++) {
			sub_arr[i].push_back(first_distri[i][j][1]);
		}
	}
}
/*
	函数作用：根据分配的区域，规划路径
	res：路径结果；
*/
void dfs(treenode* root, std::vector<int>&res, std::set<int>& is_exist) {
	if (!root) return;
	if(is_exist.find(root->val) != is_exist.end())
		is_exist.erase(root->val);
	res.push_back(root->val);
	for (int i = 0; i < root->next.size(); i++) {
		if (root->next[i]) {
			dfs(root->next[i], res, is_exist);
			if(is_exist.size())
				res.push_back(root->val);
		}
	}
}
/*
	函数作用：计算两个区域之间的路径；
	start_id：起始区域；
	end_id：目标与区
	graph：邻接图；
	end_path：路径结果
*/
void path_point_to_point(int N, int start_id, int end_id, const std::vector<std::vector<int> >& graph, std::vector<int>& end_path) {
	std::vector<int> visit(N - 1, 0);
	std::priority_queue<grid_in_Dijkstra> pq;
	std::vector<grid_in_Dijkstra> all_grid(N - 1);
	for (int i = 0; i < N - 1; i++) {
		all_grid[i].index = i;
	}
	all_grid[start_id].distance_to_ori = 0;
	pq.push(all_grid[start_id]);
	while (!pq.empty()) {
		auto tmp = pq.top();
		pq.pop();
		// if (tmp.index == end_id) break;
		if (visit[tmp.index] == 1) continue;
		visit[tmp.index] = 1;
		for (int i = 0; i < graph[tmp.index].size(); i++) {
			if (graph[tmp.index][i] != 0 && visit[i] == 0) {
				if (tmp.distance_to_ori + 1 < all_grid[i].distance_to_ori) {
					all_grid[i].distance_to_ori = tmp.distance_to_ori + 1;
					all_grid[i].pre_index = tmp.index;
				}
				pq.push(all_grid[i]);
			}
		}
	}
	grid_in_Dijkstra my_gird = all_grid[end_id];
	while (my_gird.index != start_id) {
		end_path.push_back(my_gird.index);
		my_gird = all_grid[my_gird.pre_index];
	}
	reverse(end_path.begin(), end_path.end());
}
/*
	函数作用：根据初分配的区域，构建最小生成树
	id：路径起点
	one_set：分配的区域；
	adjacency_matrix_except_endnode：去除终点后的邻接矩阵
	real_path：路径结果
*/
void path_planning(int N, int id, const std::vector<std::vector<int> >& dist, const std::vector<int>& one_set, const std::vector<std::vector<int> >& adjacency_matrix_except_endnode, std::vector<int>& real_path) {
	if (!one_set.size()) return;
	std::set<int> is_exist;
	for (int i = 0; i < one_set.size(); i++) {
		is_exist.insert(one_set[i]);
	}
	is_exist.erase(id);
	treenode* head = new treenode(id);
	head->distance_to_other_point = 0;
	for (int k = 0; k < 3; k++) {
		if (k != id) {
			head->distance_to_other_point += dist[k][id];
		}
	}
	std::queue<treenode*> q;
	q.push(head);
	while (!q.empty()) {
		auto tmp = q.front();
		q.pop();
		for (int i = 0; i < N - 1; i++) {
			if (adjacency_matrix_except_endnode[tmp->val][i] != 0 && is_exist.count(i) == 1) {
				treenode* node = new treenode(i);
				node->distance_to_other_point = 0;
				for (int k = 0; k < 3; k++) {
					if (k != id) {
						node->distance_to_other_point += dist[k][i];
					}
				}
				tmp->next.push_back(node);
			}
		}
		sort(tmp->next.begin(), tmp->next.end(), [=](treenode* a, treenode* b)->bool {
			return a->distance_to_other_point > b->distance_to_other_point;
		});
		for (int k = 0; k < tmp->next.size(); k++) {
			q.push(tmp->next[k]);
			is_exist.erase(tmp->next[k]->val);
		}
	}
	for (int i = 0; i < one_set.size(); i++) {
		is_exist.insert(one_set[i]);
	}
	dfs(head, real_path, is_exist);
}
/*
	函数作用：根据重分配的区域，生成搜寻路径
	id：路径起点
	one_set：分配的区域；
	adjacency_matrix_except_endnode：去除终点后的邻接矩阵
	real_path：路径结果
*/
void path_planning_add(int N, int id, const std::vector<int>& one_set, const std::vector<std::vector<int> >& adjacency_matrix_except_endnode, std::vector<int>& real_path) {
	if (one_set.size() <= 1) return;
	std::vector<int> to_head;
	for (int i = 0; i < one_set.size() - 1; i++) {
		std::vector<int> tmp;
		path_point_to_point(N, one_set[i], one_set[i + 1], adjacency_matrix_except_endnode, tmp);
		to_head.insert(to_head.end(), tmp.begin(), tmp.end());
	}
	real_path = to_head;
	reverse(to_head.begin(), to_head.end());
	to_head.erase(to_head.begin(), to_head.begin() + 1);
	real_path.insert(real_path.end(), to_head.begin(), to_head.end());
}
/*
	函数作用：与终点区域的路径
	id：路径起点
	graph：邻接图
	end_path：路径结果
*/
void path_to_end(int N, int start_id, const std::vector<std::vector<int> >& graph, std::vector<int>& end_path) {
	std::vector<int> visit(N, 0);
	std::priority_queue<grid_in_Dijkstra> pq;
	std::vector<grid_in_Dijkstra> all_grid(N);
	for (int i = 0; i < N; i++) {
		all_grid[i].index = i;
	}
	all_grid[start_id].distance_to_ori = 0;
	pq.push(all_grid[start_id]);
	while (!pq.empty()) {
		auto tmp = pq.top();
		pq.pop();
		if (tmp.index == N - 1) break;
		if (visit[tmp.index] == 1) continue;
		visit[tmp.index] = 1;
		for (int i = 0; i < graph[tmp.index].size(); i++) {
			if (graph[tmp.index][i] != 0 && visit[i] == 0) {
				if (tmp.distance_to_ori + 1 < all_grid[i].distance_to_ori) {
					all_grid[i].distance_to_ori = tmp.distance_to_ori + 1;
					all_grid[i].pre_index = tmp.index;
				}
				pq.push(all_grid[i]);
			}
		}
	}
	grid_in_Dijkstra my_gird = all_grid[N - 1];
	while (my_gird.index != start_id) {
		end_path.push_back(my_gird.index);
		my_gird = all_grid[my_gird.pre_index];
	}
	reverse(end_path.begin(), end_path.end());
}
/*
	函数作用：判断路径是否符合题意
*/
bool check(int N, std::vector<std::vector<int> >&real_path) {
	std::vector<int> visit(N, 0);

	std::vector<int> index(3, 0);
	std::vector<int> time(3, 0);

	while (index[0] <real_path[0].size() || index[1] <real_path[1].size() || index[2] <real_path[2].size()) {
		for (int i = 0; i < 3; i++) {
			while (index[i] < real_path[i].size() && visit[real_path[i][index[i]]] == 1) {
				index[i]++;
			}
			if (index[i] < real_path[i].size()) {
				time[i]++;
				visit[real_path[i][index[i]]] = 1;
			}
		}
	}
	int is_all_serach = 1;
	for (int i = 0; i < visit.size(); i++) {
		if (visit[i] == 0) {
			is_all_serach = 0;
			break;
		}
	}
	int min_time = std::min(time[0], std::min(time[1], time[2]));
	int max_time = std::max(time[0], std::max(time[1], time[2]));
	if (min_time * 13 < max_time * 10 || is_all_serach == 0) {
		return 0;
	} else {
		return 1;
	}
}
/*
	函数作用：生成除去终点后的图的邻接矩阵
*/
void adjacency_matrix_excep_end(int N, const std::vector<std::vector<int> >& adjacency_matrix, std::vector<std::vector<int> >& adjacency_matrix_except_endnode) {
	for (int i = 0; i < N - 1; i++) {
		for (int j = 0; j < N - 1; j++) {
			adjacency_matrix_except_endnode[i][j] = adjacency_matrix[i][j];
		}
	}
}
/*
	函数作用：判断除去终点后的图是否为强连通
*/	
bool is_strongly_connected(const std::vector<std::vector<int> >& dist) {
	int is_true = 1;
	for (int i = 0; i < dist.size(); i++) {
		for (int j = 0; j < dist[i].size(); j++) {
			if (dist[i][j] > 1e8) {
				is_true = 0;
				break;
			}
		}
	}
	return is_true;
}
/*
	函数作用：若除去终点后的图为强连通，通过该函数计算路径结果
*/
std::vector<std::vector<int> > first_case(int N, const std::vector<std::vector<int> >& dist, const std::vector<std::vector<int> >& adjacency_matrix,
	const std::vector<std::vector<int> >& adjacency_matrix_except_endnode) {

	std::vector<std::vector<std::vector<int> > >first_distri(3);
	int min_num_id = -1;
	int mid_num_id = -1;
	int max_num_id = -1;
	int min_num = 1e9;
	int mid_num = 0;
	int max_num = 0;
	first_distribution(N, dist, first_distri, min_num_id, mid_num_id, max_num_id, min_num, mid_num, max_num);
	std::vector<std::vector<int> > sub_arr(3);
	std::vector<std::vector<int> >extra_add(3);
	std::vector<std::vector<int> >real_path(3);
	std::vector<std::vector<int> >real_to_add_path(3);
	std::vector<std::vector<int> >add_path(3);
	std::vector<std::vector<int> >end_path(3);
	if (min_num * 13 >= max_num * 10) { //若初分配的结果符合题意，则直接计算并输出路径
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < first_distri[i].size(); j++) {
				sub_arr[i].push_back(first_distri[i][j][1]);
			}
		}
		for (int i = 0; i < 3; i++) {
			path_planning(N, i, dist, sub_arr[i], adjacency_matrix_except_endnode, real_path[i]);
			path_to_end(N, real_path[i].back(), adjacency_matrix, end_path[i]);
		}
		for (int i = 0; i < 3; i++) {
			real_path[i].insert(real_path[i].end(), end_path[i].begin(), end_path[i].end());
		}
	} else { //若初分配的结果不符合题意，则进行重分配
		bool can_break = 0;
		for (int add_min = 0; add_min <= max_num - min_num; add_min++) {
			if (can_break) break;
			for (int add_mid = min_num - mid_num; add_mid <= max_num - mid_num; add_mid++) {
				if (add_min + add_mid <= 0) continue;
				distribution_try(add_min, add_mid, dist, min_num_id, mid_num_id, max_num_id, min_num, mid_num, max_num,
					first_distri, sub_arr, extra_add);
				for (int i = 0; i < 3; i++) {
					path_planning(N, i, dist, sub_arr[i], adjacency_matrix_except_endnode, real_path[i]);
				}
				for (int i = 0; i < 3; i++) {
					if (extra_add[i].size()) {
						path_point_to_point(N, real_path[i].back(), extra_add[i][0], adjacency_matrix_except_endnode, real_to_add_path[i]);
						path_planning_add(N, extra_add[i][0], extra_add[i], adjacency_matrix_except_endnode, add_path[i]);
						path_to_end(N, extra_add[i][0], adjacency_matrix, end_path[i]);
					} else {
						path_to_end(N, real_path[i].back(), adjacency_matrix, end_path[i]);
					}
				}
				for (int i = 0; i < 3; i++) {
					real_path[i].insert(real_path[i].end(), real_to_add_path[i].begin(), real_to_add_path[i].end());
					real_path[i].insert(real_path[i].end(), add_path[i].begin(), add_path[i].end());
					real_path[i].insert(real_path[i].end(), end_path[i].begin(), end_path[i].end());
				}
				if (check(N, real_path)) {
					can_break = 1;
					break;
				} else {
					real_path.clear(); real_path.resize(3);
					real_to_add_path.clear(), real_to_add_path.resize(3);
					add_path.clear(), add_path.resize(3);
					end_path.clear(), end_path.resize(3);
					sub_arr.clear(); sub_arr.resize(3);
					extra_add.clear(); extra_add.resize(3);
				}
			}
		}
	}
	return real_path;
}
/*
	函数作用：若除去终点后的图不为强连通，通过该函数计算路径结果
*/
std::vector<std::vector<int>> second_case(int N, const std::vector<std::vector<int> >& dist, const std::vector<std::vector<int> >& adjacency_matrix,
	const std::vector<std::vector<int> >& adjacency_matrix_except_endnode) {
	std::vector<std::vector<int> > real_path(3);
	std::vector<std::vector<std::vector<int> > >first_distri(3);
	int min_num_id = -1;
	int mid_num_id = -1;
	int max_num_id = -1;
	int min_num = 1e9;
	int mid_num = 0;
	int max_num = 0;
	first_distribution(N, dist, first_distri, min_num_id, mid_num_id, max_num_id, min_num, mid_num, max_num);
	std::vector<std::vector<int> > sub_arr(3);
	std::vector<std::vector<int> >end_path(3);
	if (min_num * 13 >= max_num * 10) { //若初分配的结果符合题意，则直接计算并输出路径
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < first_distri[i].size(); j++) {
				sub_arr[i].push_back(first_distri[i][j][1]);
			}
		}
		for (int i = 0; i < 3; i++) {
			path_planning(N, i, dist, sub_arr[i], adjacency_matrix_except_endnode, real_path[i]);
			path_to_end(N, real_path[i].back(), adjacency_matrix, end_path[i]);
		}
		for (int i = 0; i < 3; i++) {
			real_path[i].insert(real_path[i].end(), end_path[i].begin(), end_path[i].end());
		}
	} else { //若初分配的结果不符合题意，则进行重分配
		int isolate_id;
		int isolate_num;
		for (int i = 0; i < 3; i++) {
			int tmp = 0;
			for (int j = 0; j < 3; j++) {
				if (j == i) continue;
				if (dist[i][j] > 1e8)
					tmp++;
			}
			if (tmp == 2) {
				isolate_id = i;
				isolate_num = first_distri[i].size();
			}
		}
		int small_id = -1;
		int small_num = 1e9;
		int big_id = -1;
		int big_num = 0;
		for (int i = 0; i < 3; i++) {
			if (i == isolate_id) continue;
			if (first_distri[i].size() > big_num) {
				big_num = first_distri[i].size();
				big_id = i;
			}
			if (first_distri[i].size() < small_num) {
				small_num = first_distri[i].size();
				small_id = i;
			}
		}
		int cha = big_num - small_num;
		std::vector<std::vector<int> > big_use = first_distri[big_id];
		std::vector<std::vector<int> > big_to_small;
		sort(big_use.begin(), big_use.end(), [=](std::vector<int>a, std::vector<int>b)->bool {
			return a[0] < b[0];
		});
		for (int i = 0; i < cha; i++) {
			big_to_small.push_back(big_use.back());
			big_use.pop_back();
		}
		for (int i = 0; i < big_to_small.size(); i++) {
			big_to_small[i][0] = dist[small_id][big_to_small[i][1]];
		}
		sort(big_to_small.begin(), big_to_small.end(), [=](std::vector<int>a, std::vector<int>b)->bool {
			return a[0] < b[0];
		});
		std::vector<int> small_use;
		for (int i = 0; i < big_to_small.size(); i++) {
			small_use.push_back(big_to_small[i][1]);
		}

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < first_distri[i].size(); j++) {
				sub_arr[i].push_back(first_distri[i][j][1]);
			}
		}
		for (int i = 0; i < 3; i++) {
			path_planning(N, i, dist, sub_arr[i], adjacency_matrix_except_endnode, real_path[i]);
		}

		std::vector<int> real_to_add_path;
		std::vector<int> add_path;
		path_point_to_point(N, real_path[small_id].back(), small_use[0], adjacency_matrix_except_endnode, real_to_add_path);
		path_planning_add(N, small_use[0], small_use, adjacency_matrix_except_endnode, add_path);
		path_to_end(N, small_use[0], adjacency_matrix, end_path[small_id]);

		path_to_end(N, real_path[isolate_id].back(), adjacency_matrix, end_path[isolate_id]);
		path_to_end(N, real_path[big_id].back(), adjacency_matrix, end_path[big_id]);

		real_path[isolate_id].insert(real_path[isolate_id].end(), end_path[isolate_id].begin(), end_path[isolate_id].end());
		real_path[big_id].insert(real_path[big_id].end(), end_path[big_id].begin(), end_path[big_id].end());
		real_path[small_id].insert(real_path[small_id].end(), real_to_add_path.begin(), real_to_add_path.end());
		real_path[small_id].insert(real_path[small_id].end(), add_path.begin(), add_path.end());
		real_path[small_id].insert(real_path[small_id].end(), end_path[small_id].begin(), end_path[small_id].end());
	}
	return real_path;
}
/*
	函数作用：按格式输出路径信息，输出失败则直接退出
*/
bool data_to_txt(const std::vector<std::vector<int> >&real_path, const std::string k_output_filename) {
	std::ofstream file;
	file.open(k_output_filename, std::ios::out);
	if (!file) return 0;
	for (int i = 0; i < real_path.size(); i++) {
		for (int j = 0; j < real_path[i].size(); j++) {
			if (j == real_path[i].size() - 1) {
				file << real_path[i][j] << std::endl;
			} else {
				file << real_path[i][j] << " ";
			}
		}
	}
	file.close();
	return 1;
}


int main(int argc, char* argv[]) {
    // 此为原始数据文件路径，提交时请勿修改，擅自修改会导致运行找不到文件而报错

	//const std::string k_input_filename{ argv[1] };
	//// 此为结果文件路径，提交时请勿修改，擅自修改会导致评测系统无法获取结果文件
	//const std::string k_output_filename{ argv[2] };
	const std::string k_input_filename = "input.txt";
	// 此为结果文件路径，提交时请勿修改，擅自修改会导致评测系统无法获取结果文件
	const std::string k_output_filename = "output.txt";
	int N; // n>3 //终点节点到普通节点之间必须存在出发节点

	std::vector<std::vector<int> > adjacency_matrix; // 图的邻接矩阵
	if (part_params_load(N, adjacency_matrix, k_input_filename) == 0) return 0; // 输入参数读入，读取失败则直接退出
	std::vector<std::vector<int> > dist(3, std::vector<int>(N - 1, 1e9)); // 图中所有区域与出发区域0,1,2的距离信息
	std::vector<std::vector<int> > adjacency_matrix_except_endnode(N - 1, std::vector<int>(N - 1));// 除去终点的图的邻接矩阵
	adjacency_matrix_excep_end(N, adjacency_matrix, adjacency_matrix_except_endnode); // 计算除去终点的图的邻接矩阵
	calu_dist(N, adjacency_matrix_except_endnode, dist); // 计算图中所有区域与出发区域0,1,2的距离信息
	std::vector<std::vector<int> > real_path; // 保存所有飞行器的路径信息
	if (is_strongly_connected(dist)) { // 判断除去终点后的图是否为强连通
		real_path = first_case(N, dist, adjacency_matrix, adjacency_matrix_except_endnode); // 若为强连通，进入第一种情况，计算返回路径结果
	} else {
		real_path = second_case(N, dist, adjacency_matrix, adjacency_matrix_except_endnode);// 若不为强连通，进入第一种情况，计算返回路径结果
	}
	if (data_to_txt(real_path, k_output_filename) == 0) return 0; // 按格式输出路径信息，输出失败则直接退出


    return 0;
}




