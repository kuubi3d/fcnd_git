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

void MatOps::MatInt::sub(MatOps::MatInt &m)
{

}