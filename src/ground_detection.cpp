#include "ground_detection/ground_detection.hpp"
#include <queue>

PointCloudGrid::PointCloudGrid(){

    robot_cell.row = 0;
    robot_cell.col = 0;
    robot_cell.height = 0;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Index3D idx;

                idx.x = dx;
                idx.y = dy;
                idx.z = dz;
                indices.push_back(idx);
            }
        }
    }
}

PointCloudGrid::PointCloudGrid(const GridConfig& config){

    grid_config = config;
    robot_cell.row = 0;
    robot_cell.col = 0;
    robot_cell.height = 0;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Index3D idx;

                idx.x = dx;
                idx.y = dy;
                idx.z = dz;
                indices.push_back(idx);
            }
        }
    }
}

void PointCloudGrid::clear(){
    gridCells.clear();
}

void PointCloudGrid::addPoint(const pcl::PointXYZ& point, const unsigned int index) {
    int row = static_cast<int>(std::floor(point.x / grid_config.cellSizeX));
    int col = static_cast<int>(std::floor(point.y / grid_config.cellSizeY));
    int height = static_cast<int>(std::floor(point.z / grid_config.cellSizeZ));

    if(!(row >= -grid_config.gridSizeX    && row < grid_config.gridSizeX &&
         col >= -grid_config.gridSizeY    && col < grid_config.gridSizeY &&
         height >= -grid_config.gridSizeZ  && height < grid_config.gridSizeZ)){
        return;
    }

    gridCells[row][col][height].row = row;
    gridCells[row][col][height].col = col;
    gridCells[row][col][height].height = height;
    gridCells[row][col][height].points->push_back(point);
    gridCells[row][col][height].source_indices->indices.push_back(index);
}

double PointCloudGrid::computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d planeNormal = plane.normal();

    planeNormal = orientation * planeNormal;

    planeNormal.normalize(); //just in case
    return acos(planeNormal.dot(zNormal));
}

Eigen::Vector3d PointCloudGrid::computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    /** The vector of maximum slope on a plane is the projection of (0,0,1) onto the plane.
     *  (0,0,1) is the steepest vector possible in the global frame, thus by projecting it onto
     *  the plane we get the steepest vector possible on that plane.
     */
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d planeNormal(plane.normal().normalized());
    const Eigen::Vector3d projection = zNormal - zNormal.dot(planeNormal) * planeNormal;
    return projection;
}

double PointCloudGrid::calculateDistance(const GridCell& cell1, const GridCell& cell2){

    double dx = cell1.row - cell2.row;
    double dy = cell1.col - cell2.col;
    double dz = cell1.height - cell2.height;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

int PointCloudGrid::calculateMeanHeight(const std::vector<GridCell> cells){

    // Calculate the mean height of selected cells
    double total_height = 0.0;
    for (const GridCell& cell : cells) {
        total_height += cell.height;
    }

    // Find the cell closest to the mean height
    int mean_height = std::floor(total_height / cells.size());
    return mean_height;
}

int PointCloudGrid::countGroundNeighbors(const GridCell& cell){

    int neighbors{0};
    for (int i = 0; i < 26; ++i){
        int neighborX = cell.row + indices[i].x;
        int neighborY = cell.col + indices[i].y;
        int neighborZ = cell.height + indices[i].z;

        // Check if the neighbor is within the grid boundaries
        if (neighborX >= -grid_config.gridSizeX  && neighborX < grid_config.gridSizeX &&
            neighborY >= -grid_config.gridSizeY  && neighborY < grid_config.gridSizeY &&
            neighborZ >= -grid_config.gridSizeZ  && neighborZ < grid_config.gridSizeZ){

            GridCell neighbor = gridCells[neighborX][neighborY][neighborZ];
            if (neighbor.terrain_type == TerrainType::GROUND && neighbor.points->size() > 5){
                neighbors++;
            }
        }
    }
    return neighbors;
}

GridCell PointCloudGrid::cellClosestToMeanHeight(const std::vector<GridCell>& cells, const int mean_height){

    int min_height_difference = std::numeric_limits<int>::max();
    int max_ground_neighbors = std::numeric_limits<int>::min();
    GridCell closest_to_mean_height;

    for (const GridCell& cell : cells) {

        double height_difference = std::abs(cell.height - mean_height);
        int neighbor_count = countGroundNeighbors(cell);

        if (height_difference <= min_height_difference) {
            if (neighbor_count >= max_ground_neighbors){
                closest_to_mean_height = cell;
                min_height_difference = height_difference;
                max_ground_neighbors = neighbor_count;
            }
        }
    }
    return closest_to_mean_height;
}

bool PointCloudGrid::fitPlane(GridCell& cell){

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_MSAC);
    seg.setMaxIterations(1000);
    seg.setInputCloud(cell.points);
    seg.setDistanceThreshold(grid_config.groundInlierThreshold); // Adjust this threshold based on your needs
    seg.segment(*inliers, *coefficients);

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*(cell.points), centroid);
    cell.centroid = centroid;

    std::cout << "Inliera count: " << inliers->indices.size() << std::endl;

    if (inliers->indices.size() > 5) {
        cell.inliers = inliers;
        Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        normal.normalize();
        double distToOrigin = coefficients->values[3];
        cell.plane = Eigen::Hyperplane<double, 3>(normal, distToOrigin);

        //adjust height of patch
        Eigen::ParametrizedLine<double, 3> line(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
        Eigen::Vector3d newPos =  line.intersectionPoint(cell.plane);
        if(newPos.x() > 0.0001 || newPos.y() > 0.0001)
        {
            std::cout << "TraversabilityGenerator3d: Error, adjustement height calculation is weird" << std::endl;
            return false;
        }

        if(newPos.allFinite())
        {
            //cell.height = newPos.z();
        }
        const Eigen::Vector3d slopeDir = computeSlopeDirection(cell.plane);
        cell.slope = computeSlope(cell.plane);
        cell.slopeDirection = slopeDir;
        cell.slopeDirectionAtan2 = std::atan2(slopeDir.y(), slopeDir.x());

        return true;
    }
    return false;
}

void PointCloudGrid::selectStartCell(GridCell& cell){

    double distance = calculateDistance(robot_cell, cell);
    if (distance <= grid_config.startCellDistanceThreshold) {
        // This grid cell is within the specified radius around the robot

        if (cell.row >= 0 && cell.col > 0){
            selected_cells_first_quadrant.push_back(cell);
        }
        else if (cell.row >= 0 && cell.col < 0){
            selected_cells_second_quadrant.push_back(cell);
        }
        else if (cell.row <= 0 && cell.col < 0){
            selected_cells_third_quadrant.push_back(cell);
        }
        else if (cell.row >= 0 && cell.col > 0) {
            selected_cells_fourth_quadrant.push_back(cell);
        }

    }
}

std::vector<GridCell> PointCloudGrid::getGroundCells() {

    if (gridCells.empty()){
        return ground_cells;
    }

    ground_cells.clear();
    non_ground_cells.clear();
    holes_cells.clear();
    selected_cells_first_quadrant.clear();
    selected_cells_second_quadrant.clear();
    selected_cells_third_quadrant.clear();
    selected_cells_fourth_quadrant.clear();

    for (auto& rowPair : gridCells) {
        for (auto& colPair : rowPair.second) {
            for (auto& heightPair : colPair.second) {
                GridCell& cell = heightPair.second;

                if (cell.points->size() < 5) {
                    cell.terrain_type = TerrainType::UNDEFINED;
                    continue;
                }

                if (fitPlane(cell)){
                    if (cell.slope < (grid_config.slopeThresholdDegrees * (M_PI / 180)) ){
                        cell.terrain_type = TerrainType::GROUND;
                        selectStartCell(cell);
                    }
                    else{
                        cell.terrain_type = TerrainType::OBSTACLE;
                        non_ground_cells.push_back(cell);
                    }
                }
                else{
                    cell.terrain_type = TerrainType::OBSTACLE;    
                }
            }
        }
    }

    std::queue<Index3D> q;

    if (selected_cells_first_quadrant.size() > 0){
        int cells_q1_mean_height = calculateMeanHeight(selected_cells_first_quadrant);
        GridCell closest_to_mean_height_q1 = cellClosestToMeanHeight(selected_cells_first_quadrant, cells_q1_mean_height);
        Index3D q1;
        q1.x = closest_to_mean_height_q1.row;
        q1.y = closest_to_mean_height_q1.col;
        q1.z = closest_to_mean_height_q1.height;
        q.push(q1);
    }

    if (selected_cells_second_quadrant.size() > 0){
        int cells_q2_mean_height = calculateMeanHeight(selected_cells_second_quadrant);
        GridCell closest_to_mean_height_q2 = cellClosestToMeanHeight(selected_cells_second_quadrant, cells_q2_mean_height);
        Index3D q2;
        q2.x = closest_to_mean_height_q2.row;
        q2.y = closest_to_mean_height_q2.col;
        q2.z = closest_to_mean_height_q2.height;
        q.push(q2);
    }

    if (selected_cells_third_quadrant.size() > 0){
        int cells_q3_mean_height = calculateMeanHeight(selected_cells_third_quadrant);
        GridCell closest_to_mean_height_q3 = cellClosestToMeanHeight(selected_cells_third_quadrant, cells_q3_mean_height);
        Index3D q3;
        q3.x = closest_to_mean_height_q3.row;
        q3.y = closest_to_mean_height_q3.col;
        q3.z = closest_to_mean_height_q3.height;
        q.push(q3);
    }

    if (selected_cells_fourth_quadrant.size() > 0){
        int cells_q4_mean_height = calculateMeanHeight(selected_cells_fourth_quadrant);
        GridCell closest_to_mean_height_q4 = cellClosestToMeanHeight(selected_cells_fourth_quadrant, cells_q4_mean_height);
        Index3D q4;
        q4.x = closest_to_mean_height_q4.row;
        q4.y = closest_to_mean_height_q4.col;
        q4.z = closest_to_mean_height_q4.height;
        q.push(q4);
    }

    while (!q.empty()) {

        Index3D& idx = q.front();
        q.pop();

        if (gridCells[idx.x][idx.y][idx.z].expanded == true){
            continue;
        }
        gridCells[idx.x][idx.y][idx.z].expanded = true;

        GridCell current_cell = gridCells[idx.x][idx.y][idx.z];
        ground_cells.emplace_back(current_cell);

        for (int i = 0; i < 26; ++i) {

            int neighborX = current_cell.row + indices[i].x;
            int neighborY = current_cell.col + indices[i].y;
            int neighborZ = current_cell.height + indices[i].z;

            // Check if the neighbor is within the grid boundaries
            if (neighborX >= -grid_config.gridSizeX  && neighborX < grid_config.gridSizeX &&
                neighborY >= -grid_config.gridSizeY  && neighborY < grid_config.gridSizeY &&
                neighborZ >= -grid_config.gridSizeZ  && neighborZ < grid_config.gridSizeZ) {

                GridCell neighbor = gridCells[neighborX][neighborY][neighborZ];

                if (neighbor.terrain_type == TerrainType::GROUND && !neighbor.expanded){
                    Index3D n;
                    n.x = neighbor.row;
                    n.y = neighbor.col;
                    n.z = neighbor.height;
                    q.push(n);
                }
            }
        }
    }
    return ground_cells;
}

void PointCloudGrid::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, const Eigen::Quaterniond& R_body2World){

    this->clear();
    input_cloud = input;
    orientation = R_body2World;
    unsigned int index = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = input_cloud->begin(); it != input_cloud->end(); ++it)
    {
        this->addPoint(*it,index);
        index++;
    }

    ground_cells = getGroundCells();
}


std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudGrid::segmentPoints() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_inliers(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_inliers(new pcl::PointCloud<pcl::PointXYZ>());

    // Extract points based on indices
    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;

    for (auto& cell : ground_cells){
        extract_ground.setNegative (false);
        extract_ground.setInputCloud(cell.points);
        extract_ground.setIndices(cell.inliers);
        extract_ground.filter(*ground_inliers);
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = ground_inliers->begin(); it != ground_inliers->end(); ++it)
        {
            ground_points->points.push_back(*it);
        }
        
        extract_ground.setNegative(true);
        extract_ground.filter(*non_ground_inliers);
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = non_ground_inliers->begin(); it != non_ground_inliers->end(); ++it)
        {
            non_ground_points->points.push_back(*it);
        }
    }

    for (auto& cell : non_ground_cells){
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cell.points->begin(); it != cell.points->end(); ++it)
        {
            non_ground_points->points.push_back(*it);
        }
    }
    return std::make_pair(ground_points, non_ground_points);
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudGrid::extractHoles(){
//TODO
//}
