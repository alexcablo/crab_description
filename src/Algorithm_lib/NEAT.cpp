#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include "NEAT.h"


using namespace std;


bool sameSpecies(genome genome1,genome genom2)
{
    // Computes if two genomes belong to the same specie
    float dd = DeltaDisjoint*disjoint(genome1.GenesVec,genom2.GenesVec); // Calculates the disjoint in the innovation
    float dw = DeltaWeights*weights(genome1.GenesVec,genom2.GenesVec); // Calculates the disjoint in the weights

    return ((dd+dw) < DeltaThreshold);
}

float disjoint(std::vector<gene> genes1,std::vector<gene> genes2)
{
    //calculates the disjoint value

    int length = max(findmax(genes1),findmax(genes2));    //Find the max number of innovation
    //bool i1[length]={};
    //length=40;

    bool *i1 = (bool*)calloc(length,sizeof(bool));
    //bool i2[length]={};
    bool *i2 = (bool*)calloc(length,sizeof(bool));

    float disjointGenes = 0;

    //Create 2 arrays i1 and i2 initialized to 0(false)

    //Fill each array if the innovation index is in the genome

    //For each disjoint goes up disjointGenes

    for(unsigned int i=0;i<genes1.size();++i)
    {

        i1[genes1[i].innovation-1]=true;
    }

    for(unsigned int i=0;i<genes2.size();++i)
    {
        i2[genes2[i].innovation-1]=true;

        if(!(i1[genes2[i].innovation-1]))
        {
            ++disjointGenes;
        }
    }

    for(unsigned int i=0;i<genes1.size();++i)
    {
        if(!(i2[genes1[i].innovation-1]))
        {
            ++disjointGenes;
        }
    }

    float n = max(genes1.size(),genes2.size());
    free(i1);
    free(i2);

    return disjointGenes/n;
}

int findmax(std::vector<gene> gene)
{
    //Finds the maximum innovation value
    int maxi=0;

    for(unsigned int i=0;i<gene.size();++i)
        {
            if(gene[i].innovation>maxi)
            {
                maxi=gene[i].innovation;
            }
        }

    return maxi;
}

float weights(std::vector<gene> genes1,std::vector<gene> genes2)
{
    //Return the disjoint value for the weights
    int length = max(findmax(genes1),findmax(genes2));
    //bool i2[length]={0};
    bool *i2 = (bool*)calloc(length,sizeof(bool));
    //int i2l[length]={0};
    int *i2l = (int*)calloc(length,sizeof(int));

    for(unsigned int i=0;i<genes2.size();++i)
    {
        i2[genes2[i].innovation-1]=true;
        i2l[genes2[i].innovation-1]=i;
    }

    float sum=0;
    float coincident=0;

    for(unsigned int i=0;i<genes1.size();++i)
    {
        if(genes1[i].innovation<=length)
        {
            if((i2[genes1[i].innovation-1]) == true)
            {
                sum += fabs(genes1[i].weight-genes2[i2l[genes1[i].innovation-1]].weight);
                ++coincident;
            }
        }
    }

    free(i2l);
    free(i2);

    return sum/coincident;
}

void Pool::addToSpecies(genome child)
{
    // Add a genome to a specie
    bool foundSpecie = false;
    for (int s=0;s<SpeciesVec.size();++s)
    {
        // search for a existent specie
        if (!(foundSpecie) && (sameSpecies(child, SpeciesVec[s].GenomesVec[0])))
        {
            SpeciesVec[s].GenomesVec.push_back(child);
            foundSpecie = true;
        }
    }
    if (!foundSpecie)
    {
        //If not finds a specie to fit the new genome creates a new one
        specie newspecie;
        newspecie.GenomesVec.push_back(child);
        SpeciesVec.push_back(newspecie);
    }
}

void Pool::initializePool()
{
    for(int i=0;i<Population;++i)
    {
        genome basic = basicGenome(&innovation);
        //printf("%d\n",i);
        addToSpecies(basic);
    }
}

genome basicGenome(int* innovation)
{
    genome genome;
    genome.maxneuron = Inputs-1;
    genome.mutate(innovation);

    return genome;
}

void genome::mutate(int* innovation)
{
    //Function to mutate a genome

    for (unsigned int i=0;i<7;++i)
    {
        //Modificaion of the mutation ratios
        if(RANDOM<0.5)
        {
            mutationRates[i] *= 0.98;
        }else{
            mutationRates[i] *= 1.05263;
        }
    }

    if(RANDOM<mutationRates[0])
    {
        //Choose a point mutation
        pointMutate();
    }

    for(float p=mutationRates[1];p>0;--p)
    {
        if(RANDOM<p)
        {
            //Choose a link mutation without bias
            linkMutate(false,innovation);
        }
    }

    for(float p=mutationRates[2];p>0;--p)
    {
        if(RANDOM<p)
        {
            //Choose a link mutation forcing bias
            linkMutate(true,innovation);
        }
    }

    for(float p=mutationRates[3];p>0;--p)
    {
        if(RANDOM<p)
        {
            //Chosse a node mutation
            nodeMutate(innovation);
        }
    }

    for(float p=mutationRates[4];p>0;--p)
    {
        if(RANDOM<p)
        {
            enableDisableMutate(true);
        }
    }

    for(float p=mutationRates[5];p>0;--p)
    {
        if(RANDOM<p)
        {
            enableDisableMutate(false);
        }
    }


}

void genome::pointMutate()
{
    //Changes the weight of a link
    for(unsigned int i=0;i<GenesVec.size();++i)
    {
        if(RANDOM<PerturbChance)
        {
           GenesVec[i].weight += RANDOM*mutationRates[6]*2-mutationRates[6];
        }else{
            GenesVec[i].weight += RANDOM*4-2;
        }
    }
}

void genome::linkMutate(bool forceBias,int* innovation)
{
    //Adds a new link between two existing neurons
    int neuron1 = randomNeuron(false); // Pick a random neuron of the existing ones
    int neuron2 = randomNeuron(true);

    gene newLink;
    if((neuron1<Inputs) && (neuron2<Inputs))
    {
        //Both input nodes
        return;
    }

    if (neuron2<Inputs)
    {
        //Change neuron order
        swap(neuron1,neuron2);
    }

    newLink.into = neuron2;
    newLink.out = neuron1;

    if (forceBias)
    {
        //The out is the bias
        newLink.out = Inputs-1;
    }

    if(containsLink(newLink))
    {
        return;
    }

    ++*innovation;
    newLink.innovation = *innovation;
    newLink.weight = RANDOM*4-2;

    GenesVec.push_back(newLink);
}

int genome::randomNeuron(bool nonInput)
{
    // return a random neuron form the existing ones
    //bool neurons[MaxNodes+Outputs]={};
    bool *neurons = (bool*)calloc(MaxNodes+Outputs,sizeof(bool));

    if (!nonInput)
    {
        for(int i=0;i<(Inputs-1);++i)
        {
            //Add inputs to the list
            neurons[i]=true;
        }
    }

    for(int i=0;i<=Outputs;++i)
    {
        //Add outputs to the list
        neurons[MaxNodes+i]=true;
    }

    for(unsigned int i=0;i<GenesVec.size();++i)
    {
        //Search for all the existing neurons
        if((!nonInput)||(GenesVec[i].into>=Inputs))
        {
            neurons[GenesVec[i].into]=true;
        }
        if((!nonInput)||(GenesVec[i].out>=Inputs))
        {
            neurons[GenesVec[i].out]=true;
        }
    }

    std::vector<int> neuronsl;

    for(int i=0;i<(MaxNodes+Outputs);++i)
    {
        if(neurons[i])
        {
            //Creates a the list with all the possible choicable neurons
            neuronsl.push_back(i);
        }
    }
    int randnum = rand()%(neuronsl.size());
    return neuronsl[randnum];

}

bool genome::containsLink(gene Link)
{
    //Check if the link exist in the net
    for(unsigned int i=0;i<GenesVec.size();i++)
    {
        if((GenesVec[i].into == Link.into)&&(GenesVec[i].out == Link.out))
        {
            return true;
        }
    }
    return false;
}

void genome::nodeMutate(int* innovation)
{
    //Mutates a node, choose a existing link and creates a new neuron in the link
    if(GenesVec.size()==0)
    {
        return;
    }

    int genernd = rand()%(GenesVec.size());

    if(!GenesVec[genernd].enabled)
    {
        return;
    }

    if(maxneuron==MaxNodes)
    {
        return;
    }
    ++maxneuron;

    GenesVec[genernd].enabled = false;

    //Assign the propierties of the first link
    gene gene1 = GenesVec[genernd];
    gene1.out = maxneuron;
    gene1.weight = 1;
    ++*innovation;
    gene1.innovation = *innovation;
    gene1.enabled = true;
    GenesVec.push_back(gene1);

    //Assign the propierties of the second link
    gene gene2 = GenesVec[genernd];
    gene2.into = maxneuron;
    ++*innovation;
    gene2.innovation = *innovation;
    gene2.enabled = true;
    GenesVec.push_back(gene2);
}

void genome::enableDisableMutate(bool enable)
{
    // enable or disable a link
    int cont=0;
    std::vector<int> index;
    for(unsigned int i=0;i<GenesVec.size();++i)
    {
        if(GenesVec[i].enabled==!enable)
        {
           ++cont;
           index.push_back(i);
        }
    }

    if(cont==0)
    {
        return;
    }

    int genernd = index[rand()%cont];
    GenesVec[genernd].enabled = !GenesVec[genernd].enabled;
}

void customWriteFile(Pool pool,std::string filename)
{
    ofstream file;
    file.open(filename.c_str());
    file << pool.inputsnum SPACE;
    file << pool.outputsnum SPACE;
    file << "Pool generation:" SPACE;
    file << pool.generation SPACE;
    file << pool.innovation SPACE;
    file << pool.currentSpecies SPACE;
    file << pool.currentGenome SPACE;
    file << pool.maxFitness SPACE;

    file << "Number of species:" SPACE;
    file << pool.SpeciesVec.size() SPACE;
    for(unsigned int i=0;i<pool.SpeciesVec.size();++i)
    {
        file << TAB "Specie:" << i SPACE;
        file << TAB pool.SpeciesVec[i].averageFitness SPACE;
        file << TAB pool.SpeciesVec[i].topFitness SPACE;
        file << TAB pool.SpeciesVec[i].staleness SPACE;
        file << TAB "Number of genomes:" SPACE;
        file << TAB pool.SpeciesVec[i].GenomesVec.size() SPACE;
        for(unsigned int x=0;x<pool.SpeciesVec[i].GenomesVec.size();++x)
        {
            file << TAB TAB "Genome:" << x SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].fitness SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].maxneuron SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].globalRank SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[0] SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[1] SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[2] SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[3] SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[4] SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[5] SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].mutationRates[6] SPACE;
            file << TAB TAB "Number of genes:" SPACE;
            file << TAB TAB pool.SpeciesVec[i].GenomesVec[x].GenesVec.size() SPACE;
            for(unsigned int y=0;y<pool.SpeciesVec[i].GenomesVec[x].GenesVec.size();++y)
            {
                file << TAB TAB TAB "Gene:" << y SPACE;
                file << TAB TAB TAB pool.SpeciesVec[i].GenomesVec[x].GenesVec[y].enabled SPACE;
                file << TAB TAB TAB pool.SpeciesVec[i].GenomesVec[x].GenesVec[y].innovation SPACE;
                file << TAB TAB TAB pool.SpeciesVec[i].GenomesVec[x].GenesVec[y].into SPACE;
                file << TAB TAB TAB pool.SpeciesVec[i].GenomesVec[x].GenesVec[y].out SPACE;
                file << TAB TAB TAB pool.SpeciesVec[i].GenomesVec[x].GenesVec[y].weight SPACE;
            }
        }
    }
    file.close();
}

Pool customReadFile()
{

    std::string::size_type sz;
    ifstream file;
    string line;
    file.open("TestGen30.txt");

    READ;
    int inputsval = atoi( line.c_str() );
    READ;
    int outputsval = atoi( line.c_str() );

    Pool pool(inputsval,outputsval);

    READ;

    READ;
    pool.generation = atoi( line.c_str() );
    READ;
    pool.innovation = atoi( line.c_str() );
    READ;
    pool.currentSpecies = atoi( line.c_str() );
    READ;
    pool.currentGenome = atoi( line.c_str() );
    READ;
    pool.maxFitness = atof( line.c_str() );

    READ;
    READ;
    int iter = atoi( line.c_str() );

    for(int i=0;i<iter;++i)
    {
        specie specieread;
        READ;

        READ;
        specieread.averageFitness = atof( line.c_str() );
        READ;
        specieread.topFitness = atof( line.c_str() );
        READ;
        specieread.staleness = atoi( line.c_str() );

        READ;
        READ;
        int iter2 = atoi( line.c_str() );

        pool.SpeciesVec.push_back(specieread);

        for(int x=0;x<iter2;++x)
        {
            genome genomeread;
            READ;
            READ;
            genomeread.fitness = atof( line.c_str() );
            READ;
            genomeread.maxneuron = atoi( line.c_str() );
            READ;
            genomeread.globalRank = atoi( line.c_str() );
            READ;
            genomeread.mutationRates[0] = atof( line.c_str() );
            READ;
            genomeread.mutationRates[1] = atof( line.c_str() );
            READ;
            genomeread.mutationRates[2] = atof( line.c_str() );
            READ;
            genomeread.mutationRates[3] = atof( line.c_str() );
            READ;
            genomeread.mutationRates[4] = atof( line.c_str() );
            READ;
            genomeread.mutationRates[5] = atof( line.c_str() );
            READ;
            genomeread.mutationRates[6] = atof( line.c_str() );

            READ;
            READ;
            int iter3 = atoi( line.c_str() );

            pool.SpeciesVec[i].GenomesVec.push_back(genomeread);

            for(int z=0;z<iter3;++z)
            {
                gene geneRead;
                READ;
                READ;
                geneRead.enabled = atoi( line.c_str() );
                READ;
                geneRead.innovation = atoi( line.c_str() );
                READ;
                geneRead.into = atoi( line.c_str() );
                READ;
                geneRead.out = atoi( line.c_str() );
                READ;
                geneRead.weight = atof( line.c_str() );
                pool.SpeciesVec[i].GenomesVec[x].GenesVec.push_back(geneRead);
            }
        }
    }

    file.close();

    return pool;

}

genome crossover(genome genome1,genome genome2)
{
    // create a child with two parents
    if(genome2.fitness>genome1.fitness)
    {
        swap(genome1,genome2);
    }

    genome child;

    int length = findmax(genome2.GenesVec);
    //bool innovation2[length] = {};
    bool *innovation2 = (bool*)calloc(length,sizeof(bool));
    //bool added[length] = {};
    bool *added = (bool*)calloc(length,sizeof(bool));
    //int innovation2l[length] = {};
    int *innovation2l = (int*)calloc(length,sizeof(int));

    for(unsigned int i=0;i<genome2.GenesVec.size();++i)
    {
        // Generates the list of all the innovations
        innovation2[genome2.GenesVec[i].innovation-1]=true;
        innovation2l[genome2.GenesVec[i].innovation-1]=i;
    }

    //Checks for all the coincidences and choose one of the parents, when there is no coincidence adds directly to the child the innovation
    for(unsigned int i=0;i<genome1.GenesVec.size();++i)
    {
        if(genome1.GenesVec[i].innovation<=length)
        {
            if ((innovation2[genome1.GenesVec[i].innovation-1]) && (RANDOM<=0.5) && (genome2.GenesVec[innovation2l[genome1.GenesVec[i].innovation-1]].enabled))
            {
                child.GenesVec.push_back(genome2.GenesVec[innovation2l[genome1.GenesVec[i].innovation-1]]);
                added[genome1.GenesVec[i].innovation-1]=true;
            }else{
                child.GenesVec.push_back(genome1.GenesVec[i]);
                added[genome1.GenesVec[i].innovation-1]=true;
            }
        }else{
            child.GenesVec.push_back(genome1.GenesVec[i]);
        }
    }

    for(int i=0;i<length;++i)
    {
        if(innovation2[i]&&(!added[i]))
        {
            child.GenesVec.push_back(genome2.GenesVec[innovation2l[i]]);
        }
    }

    child.maxneuron = max(genome1.maxneuron,genome2.maxneuron);

    for(unsigned int i=0;i<7;++i)
    {
        //Inherits the rates of the genome1
        child.mutationRates[i] = genome1.mutationRates[i];
    }

    free(innovation2l);
    free(innovation2);
    free(added);
    return child;
}

void Pool::cullSpecies(bool cutToOne)
{
    //Eliminates the half of genomes of each specie, or only to only 1 if it's true
    // depending of the genome performance

    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        std::vector<genome> Genomecopy;
        Genomecopy = SpeciesVec[i].GenomesVec;
        //bool visited[Genomecopy.size()]={};
        bool *visited = (bool*)calloc(Genomecopy.size(),sizeof(bool));
        int index=0;
        SpeciesVec[i].GenomesVec.clear();
        float maxFitnesslocal = 0;

        if(!cutToOne)
        {
            for(unsigned int z=0;z<ceil((double)Genomecopy.size()/2);++z)
            {
                //Search for the half top of the list of the best performance genomes
                maxFitnesslocal = 0;
                index = 0;
                for(unsigned int x=0;x<Genomecopy.size();++x)
                {
                    if(!visited[x])
                    {
                        if(maxFitnesslocal<Genomecopy[x].fitness)
                        {
                            maxFitnesslocal=Genomecopy[x].fitness;
                            index = x;
                        }
                    }
                }
                SpeciesVec[i].GenomesVec.push_back(Genomecopy[index]);
                visited[index]=true;
            }
        }

        if(cutToOne)
        {
            maxFitnesslocal = 0;
            for(unsigned int x=0;x<Genomecopy.size();++x)
            {
                //Search for the genome with highest fitness
                    if(maxFitnesslocal<Genomecopy[x].fitness)
                    {
                        maxFitnesslocal=Genomecopy[x].fitness;
                        index = x;
                    }
            }
            SpeciesVec[i].GenomesVec.push_back(Genomecopy[index]);
        }
        free(visited);
    }
}

genome specie::breedChild(int* innovation)
{
    //Generates a child for 2 random parents or copies and mutates from one parent
    genome child;
    int genernd;
    if(RANDOM < CrossoverChance)
    {
        genernd = rand()%(GenomesVec.size());
        genome g1 = GenomesVec[genernd];
        genernd = rand()%(GenomesVec.size());
        genome g2 = GenomesVec[genernd];
        child = crossover(g1,g2);
    }else{
        genernd = rand()%(GenomesVec.size());
        child = GenomesVec[genernd];
    }
    child.mutate(innovation);
    return child;
}

void Pool::removeStaleSpecies()
{
    //Remove species that have been performing bad during a few generations
    std::vector<specie> survived;
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        std::vector<genome> Genomecopy;
        Genomecopy = SpeciesVec[i].GenomesVec;

        //bool visited[Genomecopy.size()]={};
        bool *visited = (bool*)calloc(Genomecopy.size(),sizeof(bool));
        int index=0;

        SpeciesVec[i].GenomesVec.clear();
        float localmaxFitness = 0;

        //Order by max fitness
        for(unsigned int z=0;z<Genomecopy.size();++z)
        {
            localmaxFitness = 0;
            index = 0;
            for(unsigned int x=0;x<Genomecopy.size();++x)
            {
                if(!visited[x])
                {
                    if(localmaxFitness<Genomecopy[x].fitness)
                    {
                        localmaxFitness=Genomecopy[x].fitness;
                        index = x;
                    }
                }
            }
            SpeciesVec[i].GenomesVec.push_back(Genomecopy[index]);
            visited[index]=true;
        }


        if(SpeciesVec[i].GenomesVec[0].fitness >= SpeciesVec[i].topFitness)
        {
            SpeciesVec[i].topFitness = SpeciesVec[i].GenomesVec[0].fitness;
            SpeciesVec[i].staleness = 0;
        }else{
            ++SpeciesVec[i].staleness;
        }

        if((SpeciesVec[i].staleness < StaleSpecies)||(SpeciesVec[i].topFitness >= maxFitness))
        {
            survived.push_back(SpeciesVec[i]);
        }

        free(visited);
    }
    SpeciesVec.clear();

    SpeciesVec = survived;
}

void Pool::removeWeakSpecies()
{
    //Remove species that are useless
    std::vector<specie> survived;

    float sum = totalAverageFitness();
    bool breeded = false;

    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        breeded = false;
        float breed = floor(SpeciesVec[i].averageFitness/sum * Population);
        if (breed >=1)
        {
            survived.push_back(SpeciesVec[i]);
            breeded = true;
        }
        if((SpeciesVec[i].topFitness>=maxFitness)&&(!breeded))
        {
            survived.push_back(SpeciesVec[i]);
        }
    }
    if(survived.size()==0)
        survived.push_back(SpeciesVec[0]);
    SpeciesVec.clear();
    SpeciesVec = survived;
}

float Pool::totalAverageFitness()
{
    float total = 0;
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        total += SpeciesVec[i].averageFitness;
    }

    return total;
}

void specie::calculateAverageFitness()
{
    float total=0;

    for(unsigned int i=0;i<GenomesVec.size();++i)
    {
        total += GenomesVec[i].globalRank;
    }

    averageFitness = total/GenomesVec.size();
}

void Pool::rankGlobally()
{
    //Ranks ordening all the genomes
    int cont = 0;
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        //Calculate number of genomes
        cont += SpeciesVec[i].GenomesVec.size();
    }
    //bool visited[cont] ={};
    bool *visited = (bool*)calloc(cont,sizeof(bool));
    int indexspecie;
    int indexgenome;
    int indexvisited;
    int countvisited;
    float localmaxFitness;

    for(int z=cont;z>=0;--z)
    {
        localmaxFitness = 0;
        indexspecie = 0;
        indexgenome = 0;
        indexvisited = 0;
        countvisited = 0;

        for(unsigned int i=0;i<SpeciesVec.size();++i)
        {
            for(unsigned int y=0;y<SpeciesVec[i].GenomesVec.size();++y)
            {
                if(!visited[countvisited])
                {
                    if(SpeciesVec[i].GenomesVec[y].fitness>localmaxFitness)
                    {
                        localmaxFitness = SpeciesVec[i].GenomesVec[y].fitness;
                        indexgenome = y;
                        indexspecie = i;
                        indexvisited = countvisited;
                    }
                }
                ++countvisited;
            }

        }
        SpeciesVec[indexspecie].GenomesVec[indexgenome].globalRank=z;
        visited[indexvisited]=true;
    }

    free(visited);
}

void Pool::newGeneration()
{
    //bool Top = false;
    cullSpecies(false);
    rankGlobally();
    removeStaleSpecies();
    rankGlobally();
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        SpeciesVec[i].calculateAverageFitness();
    }
    removeWeakSpecies();
    float sum = totalAverageFitness();
    std::vector<genome> children;
    //Add best genome intact;
    bool found = 0;
    unsigned int x = 0;
    unsigned int y = 0;
//    if(Top)
//    {
//        while(!found)
//        {
//            if(SpeciesVec[x].GenomesVec[y].globalRank==Population-1)
//            {
//                found=true;
//                children.push_back(SpeciesVec[x].GenomesVec[y]);
//            }
//                x++;
//                y++;
//            if(SpeciesVec[x].GenomesVec.size()==y)
//                y=0;
//        }
//
//    }
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        int breed = floor(SpeciesVec[i].averageFitness/sum * Population)-1;
        for(int x=0;x<breed;++x)
        {
            children.push_back(SpeciesVec[i].breedChild(&innovation));
        }
    }
    cullSpecies(true);
    while((children.size()+SpeciesVec.size())<((unsigned int)Population))
    {
        int num = rand()%SpeciesVec.size();
        children.push_back(SpeciesVec[num].breedChild(&innovation));
    }
    cout << "OK, adding specie:" <<endl;
    cout << SpeciesVec.size() << endl;
    for(unsigned int i=0;i<children.size();++i)
    {
        cout << i;
        children[i].Network.clear();
        children[i].Networkorder.clear();
        addToSpecies(children[i]);
    }
    cout << "OK" <<endl;
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        for(unsigned int j=0;j<SpeciesVec[i].GenomesVec.size();++j)
        {
            SpeciesVec[i].GenomesVec[j].cleangenomes();
        }
    }
    generation++;
    currentGenome = 0;
    currentSpecies = 0;
}

void Pool::randomFitness()
{
    int maxfit = 0;
    for(unsigned int i=0;i<SpeciesVec.size();++i)
    {
        for(unsigned int j=0;j<SpeciesVec[i].GenomesVec.size();++j)
        {
            int ale = rand()%10000;
            if(maxfit<ale)
            {
              maxFitness = ale;
            }
            SpeciesVec[i].GenomesVec[j].fitness = ale;
        }
    }
}


void genome::generateNetwork()
{

    //bool active[MaxNodes+Outputs] = {};
    bool *active = (bool*)calloc(MaxNodes+Outputs,sizeof(bool));
    bool found = false;
    Network.clear();
    Network.resize(MaxNodes+Outputs);
    Networkorder.clear();
    int layerlimit = maxneuron-Inputs+1;

    //Generate all the nodes
    for(int i=0;i<Inputs;++i)
    {
        neuron newneuron;
        Network[i]=newneuron;
        active[i]=false;
    }
    for(int o=0;o<Outputs;++o)
    {
        neuron newneuron;
        Network[MaxNodes+o]=newneuron;
        active[MaxNodes+o]=true;
    }
    for(unsigned int i=0;i<GenesVec.size();++i)
    {
        //cout << "Generating gene: ";
        //cout << i << endl;
     if(GenesVec[i].enabled)
     {
        if(!active[GenesVec[i].out])
        {
            neuron newneuron;
            Network[GenesVec[i].out]=newneuron;
            active[GenesVec[i].out]=true;
        }

        if(!active[GenesVec[i].into])
        {
            neuron newneuron;
            Network[GenesVec[i].into]=newneuron;
            active[GenesVec[i].into]=true;
        }

        found = false;
        for(unsigned int x=0;x<Network[GenesVec[i].into].IncomingVec.size();++x)
        {
            if(GenesVec[i].out==Network[GenesVec[i].into].IncomingVec[x])
                found = true;
        }
        if(!found)
        {
            Network[GenesVec[i].into].IncomingVec.push_back(GenesVec[i].out);
            Network[GenesVec[i].into].WeightVec.push_back(GenesVec[i].weight);
        }
     }
    }
    //Initialize Order the nodes
    for(int i=Inputs;i<MaxNodes;++i)
    {
        if(active[i]){
            found = false;
            int larger = 0;
            for(unsigned int j=0;j<Network[i].IncomingVec.size();++j)
            {
                if((Network[i].IncomingVec[j]>0)&&(Network[i].IncomingVec[j]<(MaxNodes+Outputs)))
                {
                    if(Layer[Network[i].IncomingVec[j]]!=0)
                    {
                        found=true;
                        if(Layer[Network[i].IncomingVec[j]]+1>larger)
                        larger=Layer[Network[i].IncomingVec[j]]+1;
                    }
                }
            }

            if(!found)
            {
                Layer[i]=1;
            }else{
                Layer[i]=larger;
            }
        }
    }
    //Check for correct layers
    bool change = true;
    int limit = 10;
    int limitindex = 0;
    while(change&&(limitindex<limit))
    {
        change = false;
        for(int i=Inputs;i<MaxNodes;++i)
        {
            if(active[i])
            {
                found = false;
                int larger = 0;
                for(unsigned int j=0;j<Network[i].IncomingVec.size();++j)
                {
                    if((Network[i].IncomingVec[j]>Inputs)&&(Network[i].IncomingVec[j]<MaxNodes))
                    {
                        if((Layer[Network[i].IncomingVec[j]]!=0)&& Layer[Network[i].IncomingVec[j]]>=Layer[j])
                        {
                            found = true;
                            if(Layer[Network[i].IncomingVec[j]]+1>larger)
                            larger=Layer[Network[i].IncomingVec[j]]+1;
                        }
                    }
                }
                if(found)
                {
                    if(larger<layerlimit)
                    {
                        if(larger!=Layer[i])
                        {
                            Layer[i]=larger;
                            change = true;
                        }
                    }
                }
            }
        }
        limitindex++;
    }

    //Create order array from the layers
    for(int j=0;j<=layerlimit;++j)
    {
        for(int i=Inputs;i<MaxNodes;++i)
        {
            if(Layer[i]!=0)
            {
                if(Layer[i]==j)
                {
                    Networkorder.push_back(i);

                }
            }
        }
    }

    for(int o=0;o<Outputs;++o)
    {
        Networkorder.push_back(MaxNodes+o);
    }
}

void genome::cleangenomes()
{
    std::vector<gene> survided;
    for(unsigned int i=0;i<GenesVec.size();++i)
    {
        bool found =0;
        for(unsigned int j=0;j<i;++j)
        {
                if(i!=j)
                {
                    if((GenesVec[i].out==GenesVec[j].out)&&(GenesVec[i].into==GenesVec[j].into))
                        found=1;
                }
        }
        if(!found)
        {
            survided.push_back(GenesVec[i]);
        }
    }
    GenesVec.clear();
    GenesVec=survided;
}


void Pool::evaluateCurrent(float* Inputseval,float* Outputsval)
{
    //Rewrite this function to use neworders
    float sum=0;
    float valant = 0;
    genome *currentgenome = &SpeciesVec[currentSpecies].GenomesVec[currentGenome];
    if(currentgenome->Network.size()==0)
    {
        cout << "Net not generated" <<endl;
        exit(-1);
    }
    for(int i=0;i<Inputs-1;++i)
    {
        currentgenome->Network[i].value=Inputseval[i];
    }
    //cout << "enter inputs" <<endl;
    currentgenome->Network[Inputs-1].value=1;
    for(int i=Inputs;i<=(currentgenome->maxneuron);++i)
    {
        sum=0;
        valant = currentgenome->Network[i].value;
        currentgenome->Network[i].value = 0;
        //int sizei = currentgenome->Network[i].IncomingVec.size();
        for(unsigned int j=0;j<currentgenome->Network[i].IncomingVec.size();++j)
        {
            if((currentgenome->Network[i].IncomingVec[j])==i){
                sum += (valant)*(currentgenome->Network[i].WeightVec[j]);
            }else{
                sum += (currentgenome->Network[currentgenome->Network[i].IncomingVec[j]].value)*(currentgenome->Network[i].WeightVec[j]);
            }
        }
        //cout << i;
        currentgenome->Network[i].value=sinus(sum);

        //printf("Neuron out%f",sum);
    }

    //cout << "Evaluated" <<endl;
    int cont = 0;
    for(int i=MaxNodes;i<(MaxNodes+Outputs);++i)
    {
        sum=0;
        valant = currentgenome->Network[i].value;
        currentgenome->Network[i].value = 0;
        for(unsigned int j=0;j<currentgenome->Network[i].IncomingVec.size();++j)
        {
            if(currentgenome->Network[i].IncomingVec[j]==i){
                sum += valant*currentgenome->Network[i].WeightVec[j];
            }else{
                sum += (currentgenome->Network[currentgenome->Network[i].IncomingVec[j]].value)*(currentgenome->Network[i].WeightVec[j]);
            }
        }
        //cout << i;
        currentgenome->Network[i].value=sinus(sum);

        //printf("Neuron output: %f",sum);

        Outputsval[cont]=currentgenome->Network[i].value;
        cont++;
    }
}

void Pool::assignfitness(float fitness)
{
    if(fitness>maxFitness)
    maxFitness=fitness;
    SpeciesVec[currentSpecies].GenomesVec[currentGenome].fitness=fitness;
}

//float sigmoid(float value)
//{
//        float out=0;
//
//        return out=1/(1+exp(-value));
//}

float sigmoid(float value)
{
        float out=0;

        return out=value/(sqrtf(1+value*value));
}


float sinus(float value)
{

  float out=0;
  return out = sin(value);
}
//int main()
//{
//
//    srand (time(NULL));
//    Pool pool;
////    pool=customReadFile();
////    pool.SpeciesVec[0].GenomesVec[0].generateNetwork();
////
////    for(unsigned int i=0;i<MaxNodes;++i)
////    {
////        cout<< pool.SpeciesVec[0].GenomesVec[0].Layer[i];
////    }
//
//    //std::vector<int> myintVec;
//    //int tam;
//
//    //tam = myintVec.size();
//
//    //myintVec.push_back(10);
//    pool.initializePool();
//
//    for(int i=0;i<20;++i)
//    {
//        cout<< pool.generation;
//        pool.randomFitness();
//        pool.newGeneration();
//    }
//
//    customWriteFile(pool,"Test1.txt");
//
////    pool.cullSpecies(false);
////    pool.rankGlobally();s
////    pool.removeStaleSpecies();
////    pool.rankGlobally();
////        for(unsigned int i=0;i<pool.SpeciesVec.size();++i)
////    {
////        pool.SpeciesVec[i].calculateAverageFitness();
////    }
////    pool.removeWeakSpecies();
//
//    //cout << pool.innovation <<endl;
//    //cout << pool.SpeciesVec.size() <<endl;
//    //pool.SpeciesVec[0].GenomesVec[0].mutate(&pool.innovation);
//    //tam = myintVec.size();
////    customWriteFile(pool,"Test.txt");
////    pool = customReadFile();
////    customWriteFile(pool);
//    //cout << myintVec[0];
//    //cout << r;
//
//
//    return 0;
//}
