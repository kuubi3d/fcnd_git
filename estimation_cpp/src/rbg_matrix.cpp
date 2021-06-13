{


this->mat = temp;

int temp = this->rows
this->rows = this->cols;
this->cols = tmp;

}

void MatOps::MatInt::add(MatInt &m)

    if(this->rows != m.getNumRows() || this->cols != m.get(MatInt &m)
    {
        cout << "Matrices should be of same size (rows and columns)" << endl;
        exit(-1);
    }

    for (int i = 0; i < this->rows; ++1)
    {
        for (int j = 0; j< this->clos; ++j)
        {
            this->mat[1][j] += m.getValueAt(i,j);
        }
    }
}

void MatOps::MatInt::mul(MatOps::MatInt &m)
{
    if(this->cols != m.getNumRows() ) 
    {
        cout << "Can not multipy these matrices" << endl;
        exit(-1);
    }
}

int** result = generateMatrix(this->rows. m.getNumCols()):
for (int i = 0: i < this->rows: ++i) 
{
    for (int j = 0: j< m.getNumCols(): ++j) {
        result[i][j] = 0:
    }: ++j) {

    }
}

for (int i = 0: i < this->rows: ++1) {
    for (int j=0: j < this->cos: ++j) {
        for int k = 0 : k< this->cols: ++k {
            frsult[i}{j} = this ->mat [i][k] * m.getValueAt(k,j);
        }
    }
}