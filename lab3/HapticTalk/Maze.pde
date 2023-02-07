/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Bereket Guta
 * @version    V0.0.1
 * @date       24-January-2023
 * @brief      Maze creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

//import java.util.ArrayList; // import the ArrayList class
import java.util.Stack;

public class MazeGrid{

    private ArrayList<MazeCell> maze;
    private MazeCell current;
    private Stack<MazeCell> stack;
    private int seed;
    public boolean finished = false;
    public boolean walls_done = false;
	
 /**
  * Constructs a maze using a maze generation algorithm
  * https://en.wikipedia.org/wiki/Maze_generation_algorithm
  *
  * @param    cols: width of the canvas
  * @param    rows: height of the canvas
  * @param    cell_width: one side of the square cell width
  * @param    boxW: width of FBox (larger side)
  * @param    boxH: height of FBox (smaller side)
  * @param    seed: the seed for maze generation (set to -1 for random seed)
  */	
	public MazeGrid(int cols, int rows, int cell_width, float boxW, float boxH, int seed){

    // initalize the grid for the maze
    this.maze = new ArrayList<MazeCell>();
    this.stack = new Stack<MazeCell>();
    this.seed = seed;
    
    // set seed for reproducibility
    if(this.seed >= 0)
      randomSeed(this.seed);

    // fill the maze with the cells
    for (int j = 0; j < cols; j++) {
      for (int i = 0; i < rows; i++) {
        MazeCell cell = new MazeCell(i, j, cols, rows, cell_width, boxW, boxH);
        this.maze.add(cell);
      }
    } // end fill

    // set the first cell as current
    this.current = this.maze.get(0);

	}

  // draw the grid of cells with their walls
  public void show(){
    for (MazeCell cell : this.maze) {
      cell.show();
    }
  }
  
    public void drawWalls(FWorld world){
      
    if(this.finished & !walls_done){ //<>//
      for (MazeCell cell : this.maze) {
        cell.drawWalls(world);
      }
      walls_done = true;
    }
  }
  
  // step through maze creation
  // each call to this function will move through one iteration of maze generation
  public void stepMaze(){
    this.current.setVisited(true);
    color c = color(0, 226, 86, 163);
    this.current.highlight(c);
    
    // Step 1
    var next = current.checkNeighbours(this.maze);
    
    if (next != null){
      next.setVisited(true);
      
      // Step 2
      this.stack.push(current);
      
      // Step 3
      removeWalls(current, next);
      
      // Step 4
      this.current = next;  
    }
    else if (this.stack.size() > 0){
      this.current = this.stack.pop();
    } else{
      this.finished = true;
      c = color(221, 0, 0, 163);
      this.maze.get(this.maze.size() - 1).highlight(c);
    }
  }
  
  // remove the walls between the two cells
  private void removeWalls(MazeCell a, MazeCell b){
    var x = a.i - b.i;
    if (x == 1) { // b | a
      a.walls[3] = false; // remove left wall for a
      b.walls[1] = false; // remove right wall for b
    } else if (x == -1) { // a | b
      a.walls[1] = false; // remove right wall for a
      b.walls[3] = false; // remove left wall for b
    }
    var y = a.j - b.j;
    if (y == 1) {
      a.walls[0] = false; // remove top wall for a
      b.walls[2] = false; // remove bottom wall for b
    } else if (y == -1) {
      a.walls[2] = false; // remove bottom wall for a
      b.walls[0] = false; // remove top wall for b
    }
  
  }

}


// maze cell class
private class MazeCell{

  public int i,j, cols, rows, w; // top left x,y position of this cell and the col and row of the canvas as well as the cell width
  private float boxW, boxH;
  private ArrayList<FBox> boxes;
  public boolean[] walls = {true, true, true, true}; // top, right, bottom, left walls
  private boolean visited = false; // has this cell been visited yet?

  
  /**
  * Constructs a maze using a maze generation algorithm
  *
  * @param    i: x position of the cell within the canvas (top left corner)
  * @param    j: y position of the cell within the canvas (top left corner)
  * @param    cols: width of the canvas
  * @param    rows: height of the canvas
  * @param    w: one side of the square cell width
  * @param    boxW: width of FBox (larger side)
  * @param    boxH: height of FBox (smaller side)
  */
  public MazeCell(int i, int j, int cols, int rows, int w, float boxW, float boxH){
    this.i = i;
    this.j = j;
    this.cols = cols;
    this.rows = rows;
    this.w = w;
    this.boxW = boxW;
    this.boxH = boxH;
    this.boxes = new ArrayList<FBox>(); 
  }
  
  public boolean isVisited(){
    return this.visited;
  }
  
  public void setVisited(boolean value){
    this.visited = value;
  }


  // check if the neighbours have been visited and return 
  public MazeCell checkNeighbours(ArrayList<MazeCell> maze){

    ArrayList<MazeCell> neighbours  = new ArrayList<MazeCell>(); 


    // get the 4 Cells
    var top = this.index(this.i, this.j - 1);
    var right = this.index(this.i + 1, this.j);
    var bottom = this.index(this.i, this.j + 1);
    var left = this.index(this.i - 1, this.j);


    if (top != -1 && !maze.get(top).isVisited()) {
      neighbours.add(maze.get(top));
    }
    if (right != -1 && !maze.get(right).isVisited()) {
      neighbours.add(maze.get(right));
    }
    if (bottom != -1 && !maze.get(bottom).isVisited()) {
      neighbours.add(maze.get(bottom));
    }
    if (left != -1 && !maze.get(left).isVisited()) {
      neighbours.add(maze.get(left));
    }

    if (neighbours.size() > 0) {
      var r = floor(random(0, neighbours.size()));
      return neighbours.get(r);
    } else {
      return null;
    }

  }

  // get the index from the 2D position
  private int index(int i,int j){
    if (i < 0 || j < 0 || i > this.cols - 1 || j > this.rows - 1) {
      return -1;
    }

    return i + j * this.cols;
  }
  
  public void drawWalls(FWorld world){
    
    if (this.walls[0]) { // top wall
      
      var wall = new FBox(boxW,boxH);
      this.boxes.add(wall);
      
      
      var x = this.i * boxW + boxW*0.5;
      var y = this.j * boxW + boxH * 0.5;
      
      drawWall(world, wall, x, y);
      
    }
    if (this.walls[1]) { // right wall
      
      var wall = new FBox(boxH,boxW);
      this.boxes.add(wall);
      
      var x = this.i * boxW + boxW - boxH*0.5;
      var y = this.j * boxW + boxW * 0.5;
      
      
      drawWall(world, wall, x, y);
    }
    if (this.walls[2]) { // bottom wall
      var wall = new FBox(boxW,boxH);
      this.boxes.add(wall);
      
      
      var x = this.i * boxW + boxW*0.5;
      var y = this.j * boxW + boxW - boxH*0.5;

      
      drawWall(world, wall, x, y);
      //world.add(wall);
    }
    if (this.walls[3]) { // left wall
      var wall = new FBox(boxH,boxW);
      this.boxes.add(wall);
      
      var x = this.i * boxW + boxH*0.5;
      var y = this.j * boxW + boxW * 0.5;
      
     drawWall(world, wall, x, y);
    }
  }
  
  private void drawWall(FWorld world, FBox wall, float cx, float cy){
      wall.setPosition(cx, cy);
      wall.setFill(209, 201, 255, 163);
      wall.setStatic(true);
      wall.setStaticBody(true);
      wall.setGrabbable(false);
      world.add(wall);
  }


  public void show() {
    var x = this.i * w;
    var y = this.j * w;
    stroke(0);
    if (this.walls[0]) { // top wall
      line(x, y, x + this.w, y);
    }
    if (this.walls[1]) { // right wall
      line(x + this.w, y, x + this.w, y + this.w);
    }
    if (this.walls[2]) { // bottom wall
      line(x + this.w, y + this.w, x, y + this.w);
    }
    if (this.walls[3]) { // left wall
      line(x, y + this.w, x, y);
    }

    if (this.visited) {
      noStroke();
      color c = color(255, 202, 230, 50);
      fill(c);
      rect(x, y, this.w, this.w);
    }
  }
  
  public void highlight(color c) {
    var x = this.i * w;
    var y = this.j * w;
    noStroke();
    fill(c);
    rect(x, y, w, w);
  }
}
