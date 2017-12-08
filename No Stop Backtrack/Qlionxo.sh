#PBS -l nodes=1:ppn=1
#PBS -l walltime=24:00:00
#PBS -j oe

# This job's working directory
echo Working directory is $PBS_O_WORKDIR
cd $PBS_O_WORKDIR

echo " "
echo " "
echo "Job started on `hostname` at `date`"
time matlab -nodisplay < script_run
echo " "
echo "Job Ended at `date`"
echo " "
echo Job Complete

